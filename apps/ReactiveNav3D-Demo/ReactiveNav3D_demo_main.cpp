/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <iostream>
#include "ReactiveNav3D_demo.h"
#include <mrpt/system/threads.h> // sleep()
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileMemory.h>

using namespace mrpt::math;

const char *default_cfg_txt =
	"; ---------------------------------------------------------------\n"
	"; FILE: ReactiveParameters.txt\n"
	";\n"
	";  MJT @ JUNE-2013\n"
	"; ---------------------------------------------------------------\n\n\n"

	"[ROBOT_CONFIG]\n"
	";Name = MyRobot\n\n"

	"HEIGHT_LEVELS = 3 \n\n"

	";Indicate the geometry of the robot as a set of prisms\n"
	";Format - (LEVELX_HEIGHT, LEVELX_VECTORX, LEVELX_VECTORY) \n\n"

	"LEVEL1_HEIGHT = 0.6 \n"
	"LEVEL1_VECTORX = 0.3 0.3 -0.3 -0.3 \n"
	"LEVEL1_VECTORY = -0.3 0.3 0.3 -0.3 \n\n"

	"LEVEL2_HEIGHT = 0.9 \n"
	"LEVEL2_VECTORX = 0.05 0.05 -0.05 -0.05 \n"
	"LEVEL2_VECTORY = -0.1 0.1 0.1 -0.1 \n\n"

	"LEVEL3_HEIGHT = 0.6 \n"
	"LEVEL3_VECTORX = 0.30 -0.15 -0.15 \n"
	"LEVEL3_VECTORY = 0 0.2 -0.2 \n\n\n"


	"[LASER_CONFIG] \n"
	";Lasers declaration \n"
	";Indicate the number of lasers \n\n"

	"N_LASERS = 3 \n\n"

	";Indicate the lasers parameters. This information must be consistent with the robot shape (height sections). \n"
	";Laser pose is relative to the robot coordinate system. \n"
	";Information required: 	LASERX_POSE, LASERX_MAX_RANGE, LASERX_APERTURE \n"
	";							LASERX_STD_ERROR, LASERX_LEVEL, LASERX_SEGMENTS \n\n"

	"LASER1_POSE = 0 0 0.4 0 0 0 \n"
	"LASER1_MAX_RANGE = 30 \n"
	"LASER1_APERTURE = 3.141592 \n"
	"LASER1_STD_ERROR = 0.02 \n"
	"LASER1_LEVEL = 1 \n"
	"LASER1_SEGMENTS = 181 \n\n"

	"LASER2_POSE = 0 0 1.1 0 0 0 \n"
	"LASER2_MAX_RANGE = 30 \n"
	"LASER2_APERTURE = 3.141592 \n"
	"LASER2_STD_ERROR = 0.02 \n"
	"LASER2_LEVEL = 2 \n"
	"LASER2_SEGMENTS = 181 \n\n"

	"LASER3_POSE = 0 0 1.8 0 0 0 \n"
	"LASER3_MAX_RANGE = 30 \n"
	"LASER3_APERTURE = 3.141592 \n"
	"LASER3_STD_ERROR = 0.02 \n"
	"LASER3_LEVEL = 3  \n"
	"LASER3_SEGMENTS = 181 \n\n\n"


	"[KINECT_CONFIG] \n"
	";Kinects declaration \n"
	";Indicate the number of kinects \n\n"

	"N_KINECTS = 1 \n"

	";Indicate the kinect parameters. This information must be consistent with the robot shape (height sections). \n"
	";Kinect pose is relative to the robot coordinate system. \n"
	";Information required: KINECTX_LEVEL, KINECTX_X, KINECTX_Y, KINECTX_Z, KINECTX_PHI (DEGREES), KINECTX_PITCH_ANGLE (DEGREES),  \n"
	";						KINECTX_MINRANGE, KINECTX_MAXRANGE (METERS), KINECTX_ROWS, KINECTX_COLUMNS, KINECTX_STD_ERROR  \n\n"

	"KINECT1_LEVEL = 2 \n"
	"KINECT1_X = 0 \n"
	"KINECT1_Y = 0 \n"
	"KINECT1_Z = 1 \n"
	"KINECT1_PHI = 0 \n"
	"KINECT1_PITCH = 0 \n"
	"KINECT1_MINRANGE = 0.3 \n"
	"KINECT1_MAXRANGE = 5 \n"
	"KINECT1_FOV_V = 45 \n"
	"KINECT1_FOV_H = 58 \n"
	"KINECT1_ROWS = 21 \n"
	"KINECT1_COLUMNS = 21 \n"
	"KINECT1_STD_ERROR = 0.0 \n\n\n"


	"[MAP_CONFIG] \n\n"
	"; The maps are included in the ReactiveNav3D_demo.h file as .xpm files. \n"
	"; To modify them change this included files, and the method CMyReactInterface::loadmaps() if necessary \n"
	"MAP_RESOLUTION = 0.02 \n\n\n"

	"[GLOBAL_CONFIG] \n"
	"robotMax_V_mps = 0.70			; Speed limits - mps \n"
	"robotMax_W_degps = 60			; dps \n"

	"; 0: VFF,  1: ND \n"
	"HOLONOMIC_METHOD = 1 \n\n"


	"DIST_TO_TARGET_FOR_SENDING_EVENT = 0.5	; Minimum distance to target for sending the end event. Set to 0 to send it just on navigation end \n\n"

	"[ReactiveParams] \n"
	"robotMax_V_mps = 0.70			; Speed limits - mps \n"
	"robotMax_W_degps = 60			; dps \n"
	"MAX_REFERENCE_DISTANCE = 2		; Marks the maximum distance regarded by the reactive navigator (m) \n"

	"X0 = 2						; Initial location (meters) \n"
	"Y0 = 0 \n"
	"PHI0 = -90					; In degrees \n"

	";	Parameters for the navigation \n"
	"; ---------------------------------------------------- \n"
	"weights = 0.5 0.05 0.5 2.0 0.5 0.3 \n"
	"; 1: Free space \n"
	"; 2: Dist. in sectors \n"
	"; 3: Heading toward target \n"
	"; 4: Closer to target (euclidean) \n"
	"; 5: Hysteresis \n"
	"; 6: Security Distance \n\n"

	";	PTGs	.All of them has the same fields to fill, but they don't use all of them. \n"
	";----------------------------------------------------------------------------------- \n"
	"PTG_COUNT = 3			;Number of path models used \n\n"

	"PTG1_TYPE = CPTG_DiffDrive_C \n"
	"PTG1_resolution = 0.02      # Look-up-table cell size or resolution (in meters)\n"
	"PTG1_refDistance= 2.0      # Max distance to account for obstacles \n"
	"PTG1_num_paths = 121 \n"
	"PTG1_v_max_mps = 0.5 \n"
	"PTG1_w_max_dps = 45 \n"
	"PTG1_K = 1 \n"

	"PTG2_TYPE = CPTG_DiffDrive_alpha \n"
	"PTG2_resolution = 0.05      # Look-up-table cell size or resolution (in meters)\n"
	"PTG2_refDistance= 2.0      # Max distance to account for obstacles \n"
	"PTG2_num_paths = 121 \n"
	"PTG2_v_max_mps = 0.5 \n"
	"PTG2_w_max_dps = 55 \n"
	"PTG2_cte_a0v_deg = 57.3 \n"
	"PTG2_cte_a0w_deg = 57.3 \n\n"

	"PTG3_TYPE = CPTG_DiffDrive_CS  \n"
	"PTG3_resolution = 0.02      # Look-up-table cell size or resolution (in meters)\n"
	"PTG3_refDistance= 2.0      # Max distance to account for obstacles \n"
	"PTG3_num_paths = 121 \n"
	"PTG3_v_max_mps = 0.5 \n"
	"PTG3_w_max_dps = 45 \n"
	"PTG3_K = 1.0 \n"


	";	Parameters for the 'Nearness diagram' Holonomic method \n"
	"; ------------------------------------------------------------ \n"
	"[ND_CONFIG] \n"
	"factorWeights = 1.0 2.0 0.5 1.0 \n"
	"; 1: Free space \n"
	"; 2: Dist. in sectors \n"
	"; 3: Closer to target (euclidean) \n"
	"; 4: Hysteresis \n"

	"WIDE_GAP_SIZE_PERCENT = 0.25			; The robot travels nearer to obstacles if this parameter is small. \n"
	"										; The smaller it is, the closer the selected direction is respect to \n"
	"										; the Target direction in TP-Space (under some conditions) \n"
	"MAX_SECTOR_DIST_FOR_D2_PERCENT = 0.25	; \n"
	"RISK_EVALUATION_SECTORS_PERCENT = 0.25	; \n"
	"RISK_EVALUATION_DISTANCE = 0.7			; Parameter used to decrease speed if obstacles are closer than this threshold \n"
	"										; in normalized ps-meters [0,1] \n"
	"TARGET_SLOW_APPROACHING_DISTANCE = 0.8	; Used to decrease speed gradually when the target is going to be reached \n"
	"TOO_CLOSE_OBSTACLE = 0.03				; In normalized ps-meters [0,1] \n\n\n"


	";	Parameters for the VFF Holonomic method \n"
	"; ------------------------------------------------------------ \n"
	"[VFF_CONFIG] \n"
	"TARGET_SLOW_APPROACHING_DISTANCE = 0.8	; Used to decrease speed gradually when the target is going to be reached \n"
	"TARGET_ATTRACTIVE_FORCE = 7.5			; Use it to control the relative weight of the target respect to the obstacles \n\n\n"


	";	Parameters for the Obstacles grid (short term memory) \n"
	"; ------------------------------------------------------------ \n"
	"[STM_CONFIG] \n"
	"Stm_active = 1 \n			; Utilize it(1) or not(0) \n"
	"Obs_grid_length = 2		; (lenght/resolution) has to be integer \n"
	"Obs_grid_resolution = 0.04	\n"
	"Vision_limit = 0.5			; Min. Limit of vision of the RGBD sensor \n"
	"Pos_likelihood_incr = 0.8	; Range: 0.51 - 1 \n"
	"Neg_likelihood_incr = 0.4	; Range: 0 - 0.49 \n"
	"Occupancy_threshold = 0.8	; Threshold used to include or not a virtual obstacle \n";


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------


int main(int num_arg, char *argv[])
{
	try
	{
		//						Read function arguments
		//----------------------------------------------------------------------
		bool use_config_file = 0, enable_logfile = 0;
		string filename;


		if (num_arg < 2);
		else if ( string(argv[1]) == "--help")
		{
			printf("\n\t       Arguments of the function 'main' \n");
			printf("==============================================================\n\n");
			printf(" --help: Shows this menu... \n\n");
			printf(" --config FICH.txt: Load FICH.txt as config file \n\n");
			printf(" --create-config FICH.txt: Save the default config parameters \n\n");
			printf(" \t\t\t   in FICH.txt and close the program \n\n");
			printf(" --save-logfile: Enable saving a log file with navigation data \n\n");
			system::os::getch();
			return 1;
		}
		else if ( string(argv[1]) == "--create-config")
		{
			filename = argv[2];
			std::cout << std::endl << "Config_file name: " << filename;
			std::ofstream new_file(filename.c_str());
			new_file << string(default_cfg_txt);
			new_file.close();
			std::cout << std::endl << "File saved" << std::endl;
			system::os::getch();
			return 1;
		}
		else
		{
			for (int i=1; i<num_arg; i++)
			{
				if ( string(argv[i]) == "--save-logfile")
					enable_logfile = 1;

				if ( string(argv[i]) == "--config")
				{
					use_config_file = 1;
					filename = argv[i+1];
				}
			}
		}


		//Initial steps. Load configuration from file or default
		//------------------------------------------------------

		CMyReactInterface ReactInterface;
		CReactiveNavigationSystem3D rn3d (ReactInterface, true, enable_logfile);
		rn3d.enableTimeLog(true);

		if (use_config_file == 0)
		{
			utils::CConfigFileMemory configNavigation(default_cfg_txt);
			rn3d.loadConfigFile( configNavigation );
			ReactInterface.loadMaps( configNavigation );
			ReactInterface.loadConfiguration( configNavigation );
		}
		else
		{
			CConfigFile configNavigation(filename);
			rn3d.loadConfigFile( configNavigation );
			ReactInterface.loadMaps( configNavigation );
			ReactInterface.loadConfiguration( configNavigation );
		}

		ReactInterface.initializeScene();
		rn3d.initialize();


		bool stop = 0;
		bool moving_target = 0;
		int pushed_key = 0;
		TPoint3D last_Target_Pos(0,0,0);
		CTicTac	reactive_period;
		reactive_period.Tic();

		MyObserver observer;
		observer.observeBegin(ReactInterface.window);
		observer.mouse_click = 0;


		while (!stop)
		{
			if (ReactInterface.window.keyHit())
				pushed_key = ReactInterface.window.getPushedKey();
			else
				pushed_key = 0;

			switch (pushed_key) {

			case  'p':
				//Pause navigation
				rn3d.suspend();
				break;

			case 'r':
				//Resume navigation
				rn3d.resume();
				break;

			case 'm':
				//Move the target
				moving_target = 1;
				break;

			case 'e':
				//Exit program
				stop = 1;
				break;

			}

			//Set the target when the user clicks the mouse
			if (observer.mouse_click == 1)
			{
				observer.mouse_click = 0;
				if (moving_target == 1)
				{
					moving_target = 0;
					const CAbstractNavigator::TNavigationParams  nav_params = ReactInterface.createNewTarget(last_Target_Pos.x, last_Target_Pos.y, 0.3f, 0);
					rn3d.navigate(&nav_params);
				}
			}

			//Execute navigation
			rn3d.navigationStep();
			ReactInterface.robotSim.simulateOneTimeStep( reactive_period.Tac() );
			reactive_period.Tic();

			if ((rn3d.IDLE == rn3d.getCurrentState())||(rn3d.SUSPENDED == rn3d.getCurrentState()))
			{
				CSimplePointsMap auxpoints;
				mrpt::system::TTimeStamp auxpoints_time;
				ReactInterface.senseObstacles( auxpoints, auxpoints_time);
			}
			ReactInterface.updateScene();
			mrpt::system::sleep(5);


			//Move target with the mouse
			if (moving_target == 1)
			{
				int mouse_x,mouse_y;
				if (ReactInterface.window.getLastMousePosition(mouse_x,mouse_y))
				{
					// Get the ray in 3D for the latest mouse (X,Y):
					math::TLine3D ray;
					ReactInterface.scene->getViewport("main")->get3DRayForPixelCoord(mouse_x,mouse_y,ray);

					// Create a 3D plane, e.g. Z=0
					const math::TPlane ground_plane(TPoint3D(0,0,0),TPoint3D(1,0,0),TPoint3D(0,1,0));

					// Intersection of the line with the plane:
					math::TObject3D inters;
					math::intersect(ray,ground_plane, inters);

					// Interpret the intersection as a point, if there is an intersection:
					if (inters.getPoint(last_Target_Pos))
					{
						// Move an object to the position picked by the user:
						ReactInterface.scene->getByClass<CDisk>(0)->setLocation(last_Target_Pos.x, last_Target_Pos.y, last_Target_Pos.z);
					}
				}
			}

		}

		return 0;
	}
	catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
