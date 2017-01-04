/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: Rao-Blackwellized Particle Filter SLAM
	FILE: rbpf-slam.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	See README.txt for instructions or
         http://www.mrpt.org/Application:rbpf-slam
  ---------------------------------------------------------------*/

#include <mrpt/slam/CMetricMapBuilderRBPF.h>

#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPosePDFGaussian.h>

#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

/*****************************************************
			Config params
 *****************************************************/
std::string  INI_FILENAME;
std::string  RAWLOG_FILE;
unsigned int rawlog_offset;
std::string  OUT_DIR_STD;
const char *OUT_DIR;
int  LOG_FREQUENCY;
bool GENERATE_LOG_JOINT_H;
bool GENERATE_LOG_INFO;
bool SAVE_POSE_LOG;
bool SAVE_MAP_IMAGES;
bool SAVE_3D_SCENE;
bool CAMERA_3DSCENE_FOLLOWS_ROBOT;

bool SHOW_PROGRESS_IN_WINDOW;
int  SHOW_PROGRESS_IN_WINDOW_DELAY_MS;
int  PROGRESS_WINDOW_WIDTH=600, PROGRESS_WINDOW_HEIGHT=500;

std::string         METRIC_MAP_CONTINUATION_GRIDMAP_FILE; // .gridmap file
mrpt::math::TPose2D METRIC_MAP_CONTINUATION_START_POSE;

// Forward declaration.
void MapBuilding_RBPF();


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" rbpf-slam - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2)
		{
			printf("Usage: %s <config_file.ini> [<dataset.rawlog>]\n\n",argv[0]);
			mrpt::system::pause();
			return -1;
		}

		INI_FILENAME = std::string( argv[1] );
		ASSERT_FILE_EXISTS_(INI_FILENAME)

		string override_rawlog_file; 
		if (argc>=3)
			override_rawlog_file = string(argv[2]);


		CConfigFile		iniFile( INI_FILENAME );

		// ------------------------------------------
		//			Load config from file:
		// ------------------------------------------
		RAWLOG_FILE			 = !override_rawlog_file.empty() ? override_rawlog_file : iniFile.read_string("MappingApplication","rawlog_file","",  /*Force existence:*/ true);
		rawlog_offset		 = iniFile.read_int("MappingApplication","rawlog_offset",0);
		OUT_DIR_STD			 = iniFile.read_string("MappingApplication","logOutput_dir","log_out",  /*Force existence:*/ true);
		LOG_FREQUENCY		 = iniFile.read_int("MappingApplication","LOG_FREQUENCY",5,  /*Force existence:*/ true);
		GENERATE_LOG_JOINT_H = iniFile.read_bool("MappingApplication","GENERATE_LOG_JOINT_H", false);
		GENERATE_LOG_INFO    = iniFile.read_bool("MappingApplication","GENERATE_LOG_INFO", false);
		SAVE_POSE_LOG		 = iniFile.read_bool("MappingApplication","SAVE_POSE_LOG", false);
		SAVE_MAP_IMAGES		 = iniFile.read_bool("MappingApplication","SAVE_MAP_IMAGES", false);
		SAVE_3D_SCENE        = iniFile.read_bool("MappingApplication","SAVE_3D_SCENE", false);
		CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication","CAMERA_3DSCENE_FOLLOWS_ROBOT", true);
		SHOW_PROGRESS_IN_WINDOW = iniFile.read_bool("MappingApplication","SHOW_PROGRESS_IN_WINDOW", false);
		SHOW_PROGRESS_IN_WINDOW_DELAY_MS = iniFile.read_int("MappingApplication","SHOW_PROGRESS_IN_WINDOW_DELAY_MS",1);
		METRIC_MAP_CONTINUATION_GRIDMAP_FILE = iniFile.read_string("MappingApplication","METRIC_MAP_CONTINUATION_GRIDMAP_FILE","");

		METRIC_MAP_CONTINUATION_START_POSE.x = iniFile.read_double("MappingApplication","METRIC_MAP_CONTINUATION_START_POSE_X",.0);
		METRIC_MAP_CONTINUATION_START_POSE.y = iniFile.read_double("MappingApplication","METRIC_MAP_CONTINUATION_START_POSE_Y",.0);
		METRIC_MAP_CONTINUATION_START_POSE.phi = DEG2RAD( iniFile.read_double("MappingApplication","METRIC_MAP_CONTINUATION_START_POSE_PHI_DEG",.0) );

		MRPT_LOAD_CONFIG_VAR(PROGRESS_WINDOW_WIDTH, int,  iniFile, "MappingApplication");
		MRPT_LOAD_CONFIG_VAR(PROGRESS_WINDOW_HEIGHT, int,  iniFile, "MappingApplication");

		// easier!
		OUT_DIR = OUT_DIR_STD.c_str();

		// Print params:
		printf("Running with the following parameters:\n");
		printf(" RAWLOG file:'%s'\n", RAWLOG_FILE.c_str());
		printf(" Output directory:\t\t\t'%s'\n",OUT_DIR);
		printf(" Log record freq:\t\t\t%u\n",LOG_FREQUENCY);
		printf("  GENERATE_LOG_JOINT_H:\t\t\t%c\n", GENERATE_LOG_JOINT_H ? 'Y':'N');
		printf("  GENERATE_LOG_INFO:\t\t\t%c\n", GENERATE_LOG_INFO ? 'Y':'N');
		printf("  SAVE_MAP_IMAGES:\t\t\t%c\n", SAVE_MAP_IMAGES ? 'Y':'N');
		printf("  SAVE_3D_SCENE:\t\t\t%c\n", SAVE_3D_SCENE ? 'Y':'N');
		printf("  SAVE_POSE_LOG:\t\t\t%c\n", SAVE_POSE_LOG ? 'Y':'N');
		printf("  CAMERA_3DSCENE_FOLLOWS_ROBOT:\t%c\n",CAMERA_3DSCENE_FOLLOWS_ROBOT ? 'Y':'N');
		printf("  SHOW_PROGRESS_IN_WINDOW:\t%c\n",SHOW_PROGRESS_IN_WINDOW ? 'Y':'N');
		printf("\n");

		// Checks:
		ASSERT_(RAWLOG_FILE.size()>0);
		ASSERT_FILE_EXISTS_(RAWLOG_FILE)

		// Set relative path for externally-stored images in rawlogs:
		string	rawlog_images_path = extractFileDirectory( RAWLOG_FILE );
		rawlog_images_path+="/Images";
		CImage::IMAGES_PATH_BASE = rawlog_images_path;		// Set it.

		// Run:
		MapBuilding_RBPF();

		//pause();
		return 0;
	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl << "Program finished for an exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		std::cerr << "Untyped exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
}

// ------------------------------------------------------
//					MapBuilding RBPF
// ------------------------------------------------------
void MapBuilding_RBPF()
{
	MRPT_START

	CTicTac								tictac,tictacGlobal,tictac_JH;
	int									step = 0;
	CSimpleMap				finalMap;
	float								t_exec;
	COccupancyGridMap2D::TEntropyInfo	entropy;

	char								strFil[1000];

	size_t								rawlogEntry = 0;
	CFileGZInputStream					rawlogFile( RAWLOG_FILE );

	// ---------------------------------
	//		MapPDF opts
	// ---------------------------------
	CMetricMapBuilderRBPF::TConstructionOptions		rbpfMappingOptions;

	rbpfMappingOptions.loadFromConfigFile(CConfigFile(INI_FILENAME),"MappingApplication");
	rbpfMappingOptions.dumpToConsole();

	// ---------------------------------
	//		Constructor
	// ---------------------------------
	CMetricMapBuilderRBPF mapBuilder( rbpfMappingOptions );

	// handle the case of metric map continuation
	if ( !METRIC_MAP_CONTINUATION_GRIDMAP_FILE.empty() )
	{
		CSimpleMap       dummySimpleMap;
		CPosePDFGaussian startPose;

		startPose.mean.x( METRIC_MAP_CONTINUATION_START_POSE.x );
		startPose.mean.y( METRIC_MAP_CONTINUATION_START_POSE.y );
		startPose.mean.phi( METRIC_MAP_CONTINUATION_START_POSE.phi );
		startPose.cov.setZero();

		mrpt::maps::COccupancyGridMap2D gridmap;
		{
			mrpt::utils::CFileGZInputStream f(METRIC_MAP_CONTINUATION_GRIDMAP_FILE);
			f >> gridmap;
		}

		mapBuilder.initialize(dummySimpleMap,&startPose);

		for (CMultiMetricMapPDF::CParticleList::iterator it=mapBuilder.mapPDF.m_particles.begin();it!=mapBuilder.mapPDF.m_particles.end();++it) {
			CRBPFParticleData* part_d = it->d.get();
			CMultiMetricMap &mmap = part_d->mapTillNow;
			mrpt::maps::COccupancyGridMap2DPtr it_grid = mmap.getMapByClass<mrpt::maps::COccupancyGridMap2D>();
			ASSERTMSG_(it_grid.present(), "No gridmap in multimetric map definition, but metric map continuation was set (!)" );
			it_grid->copyMapContentFrom( gridmap );
		}
	}

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	//mapBuilder.setVerbosityLevel(  mrpt::utils::LVL_DEBUG );  // default value: as loaded from config file
	mapBuilder.options.enableMapUpdating		= true;
	mapBuilder.options.debugForceInsertion		= false;

	randomGenerator.randomize();

	// Prepare output directory:
	// --------------------------------
	//os::sprintf(strFil,1000,"%s/*.*",OUT_DIR);
	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	string OUT_DIR_MAPS= format("%s/maps",OUT_DIR);
	string OUT_DIR_3D= format("%s/3D",OUT_DIR);

	deleteFilesInDirectory(OUT_DIR_MAPS);
	createDirectory(OUT_DIR_MAPS);

	deleteFilesInDirectory(OUT_DIR_3D);
	createDirectory(OUT_DIR_3D);


	// Open log files:
	// ----------------------------------
	CFileOutputStream   f_log (format("%s/log_times.txt",OUT_DIR));
	CFileOutputStream   f_info (format("%s/log_info.txt",OUT_DIR));
	CFileOutputStream   f_jinfo (format("%s/log_jinfo.txt",OUT_DIR));
	CFileOutputStream   f_path (format("%s/log_estimated_path.txt",OUT_DIR));
	CFileOutputStream   f_pathOdo (format("%s/log_odometry_path.txt",OUT_DIR));
	CFileOutputStream   f_partStats (format("%s/log_ParticlesStats.txt",OUT_DIR));

	f_log.printf(
		"%% time_step  execution_time(ms)  map_size(#frames)  frame_inserted? \n"
		"%%-------------------------------------------------------------------\n");

	f_info.printf(
		"%% EMI    H    EMMI    effecMappedArea  effecMappedCells \n"
		"%%-------------------------------------------------------\n");

	f_pathOdo.printf(
		"%% time_step  x  y z yaw pitch roll timestamp \n"
		"%%--------------------------------------------\n");

	f_pathOdo.printf(
		"%% time_step  x  y z yaw pitch roll \n"
		"%%----------------------------------\n");

	f_partStats.printf(
		"%% time_step   #particles   ESS \n"
		"%%------------------------------\n");


	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CActionCollectionPtr					action;
	CSensoryFramePtr						observations;
	std::deque<CObservationGasSensorsPtr>	gasObservations;
	std::deque<CObservationWirelessPowerPtr>	wifiObservations;
	CPose3D  odoPose(0,0,0);

	CDisplayWindow3D    *win3D = NULL;

    if (SHOW_PROGRESS_IN_WINDOW)
    {
		win3D = new CDisplayWindow3D("RBPF-SLAM @ MRPT C++ Library", PROGRESS_WINDOW_WIDTH, PROGRESS_WINDOW_HEIGHT);
		win3D->setCameraZoom(40);
		win3D->setCameraAzimuthDeg(-50);
		win3D->setCameraElevationDeg(70);
    }


	tictacGlobal.Tic();
	for (;;)
	{
		if (os::kbhit())
		{
			char c = os::getch();
			if (c==27)
				break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::readActionObservationPair( rawlogFile, action, observations, rawlogEntry) )
			break; // file EOF

		if (rawlogEntry>=rawlog_offset)
		{
			// Update odometry:
			{
				CActionRobotMovement2DPtr act= action->getBestMovementEstimation();
				if (act)
					odoPose = odoPose + CPose3D( act->poseChange->getMeanVal() );
				else
				{
					CActionRobotMovement3DPtr act3D = action->getActionByClass<CActionRobotMovement3D>();
					if (act3D)
						odoPose = odoPose + act3D->poseChange.mean;
				}
			}

			double observations_timestamp_double=0;
			if (observations && observations->size()>0)
				observations_timestamp_double = mrpt::system::timestampTotime_t( (*observations->begin())->timestamp );

			// Execute:
			// ----------------------------------------
			tictac.Tic();
				mapBuilder.processActionObservation( *action, *observations );
			t_exec = tictac.Tac();
			printf("Map building executed in %.03fms\n", 1000.0f*t_exec );


			// Info log:
			// -----------
			f_log.printf("%u %f %i %i\n",
				static_cast<unsigned int>(step),
				1000.0f*t_exec,
				mapBuilder.getCurrentlyBuiltMapSize(),
				mapBuilder.m_statsLastIteration.observationsInserted ? int(1):int(0)
				);

			CPose3DPDFPtr curPDFptr = mapBuilder.getCurrentPoseEstimation();
			CPose3DPDFParticles	curPDF;

			if ( IS_CLASS( curPDFptr, CPose3DPDFParticles ) )
			{
				CPose3DPDFParticlesPtr pp= CPose3DPDFParticlesPtr(curPDFptr);
				curPDF = *pp;
			}

			if (0==(step % LOG_FREQUENCY))
			{
				const CMultiMetricMap *mostLikMap = mapBuilder.mapPDF.getCurrentMostLikelyMetricMap();

				if (GENERATE_LOG_INFO)
				{
					printf("Saving info log information...");

					tictac_JH.Tic();

					const CMultiMetricMap * avrMap = mapBuilder.mapPDF.getAveragedMetricMapEstimation();
					ASSERT_(avrMap->m_gridMaps.size()>0 );
					COccupancyGridMap2DPtr grid = avrMap->m_gridMaps[0];
					grid->computeEntropy( entropy );

					grid->saveAsBitmapFile(format("%s/EMMI_gridmap_%03u.bmp",OUT_DIR,step));

					f_info.printf("%f %f %f %f %lu\n",
						entropy.I,
						entropy.H,
						entropy.mean_I,
						entropy.effectiveMappedArea,
						entropy.effectiveMappedCells);
					printf("Ok\n EMI = %.04f    EMMI=%.04f (in %.03fms)\n",entropy.I, entropy.mean_I,1000.0f*tictac_JH.Tac());
				}

				// Pose log:
				// -------------
				if (SAVE_POSE_LOG)
				{
					printf("Saving pose log information...");
					curPDF.saveToTextFile( format("%s/mapbuild_posepdf_%03u.txt",OUT_DIR,step) );
					printf("Ok\n");
				}

				// Map images:
				// -------------
				if (SAVE_MAP_IMAGES)
				{
					printf("Saving map images to files...");

					//  Most likely maps:
					// ----------------------------------------
					mostLikMap->saveMetricMapRepresentationToFile( format("%s/mapbuilt_%05u_",OUT_DIR_MAPS.c_str(),step) );

					if (mostLikMap->m_gridMaps.size()>0)
					{
						CImage		img;
						mapBuilder.drawCurrentEstimationToImage( &img );
						img.saveToFile(format("%s/mapping_%05u.png",OUT_DIR,step));
					}

					printf("Ok!\n");
				}

				// Save a 3D scene view of the mapping process:
                COpenGLScenePtr scene;
				if (SAVE_3D_SCENE || SHOW_PROGRESS_IN_WINDOW)
				{
				    scene = COpenGLScene::Create();

					// The ground:
					mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
					groundPlane->setColor(0.4,0.4,0.4);
					scene->insert( groundPlane );

					// The camera pointing to the current robot pose:
					if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
					{
						mrpt::opengl::CCameraPtr objCam = mrpt::opengl::CCamera::Create();
						CPose3D		robotPose;
						curPDF.getMean(robotPose);

						objCam->setPointingAt(robotPose);
						objCam->setAzimuthDegrees(-30);
						objCam->setElevationDegrees(30);
						scene->insert( objCam );
					}
					// Draw the map(s):
					mrpt::opengl::CSetOfObjectsPtr objs = mrpt::opengl::CSetOfObjects::Create();
					mostLikMap->getAs3DObject( objs );
					scene->insert( objs );

					// Draw the robot particles:
					size_t		M = mapBuilder.mapPDF.particlesCount();
					mrpt::opengl::CSetOfLinesPtr objLines = mrpt::opengl::CSetOfLines::Create();
					objLines->setColor(0,1,1);
					for (size_t i=0;i<M;i++)
					{
						std::deque<TPose3D>		path;
						mapBuilder.mapPDF.getPath(i,path);

						float	x0=0,y0=0,z0=0;
						for (size_t k=0;k<path.size();k++)
						{
							objLines->appendLine(
								x0, y0, z0+0.001,
								path[k].x, path[k].y, path[k].z+0.001 );
							x0=path[k].x;
							y0=path[k].y;
							z0=path[k].z;
						}
					}
					scene->insert( objLines );

					// An ellipsoid:
					CPose3D			lastMeanPose;
					float			minDistBtwPoses=-1;
					std::deque<TPose3D>		dummyPath;
					mapBuilder.mapPDF.getPath(0,dummyPath);
					for (int k=(int)dummyPath.size()-1;k>=0;k--)
					{
						CPose3DPDFParticles	poseParts;
						mapBuilder.mapPDF.getEstimatedPosePDFAtTime(k,poseParts);

						CPose3D		meanPose;
						CMatrixDouble66 COV;
						poseParts.getCovarianceAndMean(COV,meanPose);

						if ( meanPose.distanceTo(lastMeanPose)>minDistBtwPoses )
						{
							CMatrixDouble33 COV3 = COV.block(0,0,3,3);

							minDistBtwPoses = 6 * sqrt(COV3(0,0)+COV3(1,1));

							opengl::CEllipsoidPtr objEllip = opengl::CEllipsoid::Create();
							objEllip->setLocation(meanPose.x(), meanPose.y(), meanPose.z() + 0.001 );
							objEllip->setCovMatrix(COV3, COV3(2,2)==0 ? 2:3 );

							objEllip->setColor(0,0,1);
							objEllip->enableDrawSolid3D(false);
							scene->insert( objEllip );

							lastMeanPose = meanPose;
						}
					}
				} // end if show or save 3D scene->

                if (SAVE_3D_SCENE)
                {	// Save as file:
					CFileGZOutputStream(format("%s/buildingmap_%05u.3Dscene",OUT_DIR_3D.c_str(),step)) << *scene;
				}

                if (SHOW_PROGRESS_IN_WINDOW)
                {
                    COpenGLScenePtr &scenePtr = win3D->get3DSceneAndLock();
                    scenePtr = scene;
                    win3D->unlockAccess3DScene();

                    win3D->forceRepaint();
                    int add_delay = SHOW_PROGRESS_IN_WINDOW_DELAY_MS - t_exec*1000;
                    if (add_delay>0)
                        sleep(add_delay);
                }
                /*else
                {
                    // Free scene:
                    delete scene;
                    scene=NULL;
                }
				*/


			// Save the weighted entropy of each map:
			// ----------------------------------------
				if (GENERATE_LOG_JOINT_H)
				{
					printf("Saving joint H...");
					tictac_JH.Tic();

					double  H_joint = mapBuilder.getCurrentJointEntropy();
					double  H_path  = mapBuilder.mapPDF.getCurrentEntropyOfPaths();
					f_jinfo.printf("%e %e\n",H_joint,H_path);
					printf("Ok\t joint-H=%f\t(in %.03fms)\n",H_joint,1000.0f*tictac_JH.Tac());
				}

			} // end of LOG_FREQ


			// Save the memory usage:
			// ------------------------------------------------------------------
			{
				printf("Saving memory usage...");
				unsigned long	memUsage = getMemoryUsage();
				FILE		*f=os::fopen(format("%s/log_MemoryUsage.txt",OUT_DIR).c_str(),"at");
				if (f)
				{
					os::fprintf(f,"%u\t%lu\n",step,memUsage);
					os::fclose(f);
				}
				printf("Ok! (%.04fMb)\n", ((float)memUsage)/(1024*1024) );
			}

			// Save the parts stats:
			f_partStats.printf("%u %u %f\n",
					(unsigned int)step,
					(unsigned int)curPDF.size(),
					curPDF.ESS()
					);

			// Save the robot estimated pose for each step:
			CPose3D   meanPose;
			mapBuilder.getCurrentPoseEstimation()->getMean(meanPose);

			f_path.printf("%i %f %f %f %f %f %f %f\n",
				(int)rawlogEntry,
				meanPose.x(), meanPose.y(), meanPose.z(),
				meanPose.yaw(), meanPose.pitch(), meanPose.roll(),
				observations_timestamp_double
				 );

			f_pathOdo.printf("%i\t%f\t%f\t%f\t%f\t%f\t%f\n", step, odoPose.x(), odoPose.y(), odoPose.z(), odoPose.yaw(), odoPose.pitch(), odoPose.roll() );

		} // end of if "rawlog_offset"...

		step++;
		printf("\n---------------- STEP %u | RAWLOG ENTRY %u ----------------\n",step, (unsigned)rawlogEntry);

		// Free memory:
		action.clear_unique();
		observations.clear_unique();
	}; // end while

	printf("\n---------------- END!! (total time: %.03f sec) ----------------\n",tictacGlobal.Tac());

	// Save map:
	mapBuilder.getCurrentlyBuiltMap(finalMap);

	CFileOutputStream		filOut(format("%s/_finalmap_.simplemap",OUT_DIR));
	filOut << finalMap;

	// Save gridmap extend (if exists):
	const CMultiMetricMap *mostLikMap = mapBuilder.mapPDF.getCurrentMostLikelyMetricMap();
	if (mostLikMap->m_gridMaps.size()>0)
	{
		CMatrix		auxMat(1,4);
		auxMat(0,0) = mostLikMap->m_gridMaps[0]->getXMin();
		auxMat(0,1) = mostLikMap->m_gridMaps[0]->getXMax();
		auxMat(0,2) = mostLikMap->m_gridMaps[0]->getYMin();
		auxMat(0,3) = mostLikMap->m_gridMaps[0]->getYMax();
		auxMat.saveToTextFile(format("%s/finalGridmapSize.txt",OUT_DIR),MATRIX_FORMAT_FIXED);
	}

	// Save the most likely path of the particle set
	FILE *f_pathPart;

	os::sprintf(strFil,1000,"%s/most_likely_path.txt",OUT_DIR);
	f_pathPart = os::fopen(strFil,"wt");

	ASSERT_( f_pathPart != NULL );

	std::deque<TPose3D>				outPath;
	std::deque<TPose3D>::iterator	itPath;

	mapBuilder.getCurrentMostLikelyPath( outPath );

	for( itPath = outPath.begin(); itPath != outPath.end(); itPath++ )
		os::fprintf( f_pathPart, "%.3f %.3f %.3f\n", itPath->x, itPath->y, itPath->yaw);

	os::fclose(f_pathPart);

	// Free gas readings memory (if any):
	gasObservations.clear();

	// Free wifi readings memory (if any):
	wifiObservations.clear();

	// Close 3D window, if any:
	if (win3D)
	{
	    mrpt::system::pause();
	    delete win3D;
	    win3D=NULL;
	}

	MRPT_END
}
