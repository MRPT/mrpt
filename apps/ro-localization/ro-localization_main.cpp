/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


/*---------------------------------------------------------------------------------
 APPLICATION: Range-Only Localization with Particle Filter
 AUTHORs: The MAPIR group

 DESCRIPTION: Localization with PF, with an extended model to
              cope with unknown, dynamic biases in the beacon
			  ranges. Refer to the paper: http://www.mrpt.org/paperuwb-particle-filter-localization/
 Demo config file: https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/ro-localization/UWB_localization_demo.ini
 
 Antonio J. Ortiz de Galisteo worked hard in the first version of this program.
  Gracias!! :-)
 --------------------------------------------------------------------------------- */

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileStream.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/system/threads.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/random.h>

#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CGridPlaneXY.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::bayes;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

#include "CPosePDFParticlesExtended.h"

#if MRPT_HAS_OPENGL_GLUT && MRPT_HAS_WXWIDGETS
#	define SHOW_REAL_TIME_3D
#endif

#define STORE_3D

// ------------------------------------------------------
//				Configuration
// ------------------------------------------------------
CConfigFile		*iniFile = NULL;
std::string		iniFileName;

//extern double	likelihood_acumulation;

// ------------------------------------------------------
//				TestParticlesLocalization
// ------------------------------------------------------
void TestParticlesLocalization()
{
	// Load configuration!
	// ------------------------------------------

	// PF-algorithm Options:
	// ---------------------------
	CParticleFilter::TParticleFilterOptions		pfOptions;
	pfOptions.loadFromConfigFile( *iniFile, "PF_options" );

	int				NUM_REPS			= iniFile->read_int("ro-localization","experimentRepetitions",1);
	std::string		RAWLOG_FILE			= iniFile->read_string("ro-localization","rawlog_file","NOT FOUND!");
	std::string		MAP_FILE			= iniFile->read_string("ro-localization","gridmap_file","NOT FOUND!");
	std::string		OUT_DIR_PREFIX		= iniFile->read_string("ro-localization","logOutput_dir","RO-LOCALIZATION_OUT");
	int				SCENE3D_FREQ		= iniFile->read_int("ro-localization","3DSceneFrequency",10);
	std::string		GT_FILE				= iniFile->read_string("ro-localization","groundTruthFile","");

	CMatrix		aux(1,1);

	float  Pc_range_ini=0.05f;
	float  Pc_range_end=0.05f;
	float  Pc_range_step=0.05f;

	MRPT_LOAD_CONFIG_VAR( Pc_range_ini,  float,  (*iniFile), "ro-localization" )
	MRPT_LOAD_CONFIG_VAR( Pc_range_end,  float,  (*iniFile), "ro-localization" )
	MRPT_LOAD_CONFIG_VAR( Pc_range_step, float,  (*iniFile), "ro-localization" )

	bool	SHOW_3D_FRANCO_POSITION=false;
	bool	SAVE_3D_TO_VIDEO=false;

	MRPT_LOAD_CONFIG_VAR(SHOW_3D_FRANCO_POSITION, bool, (*iniFile), "ro-localization");
	MRPT_LOAD_CONFIG_VAR(SAVE_3D_TO_VIDEO, bool, (*iniFile), "ro-localization");


	ASSERT_FILE_EXISTS_(RAWLOG_FILE)

	// Load GT:
	CMatrix			groundTruth;
	if (!GT_FILE.empty())
		groundTruth.loadFromTextFile( GT_FILE );

	// Real ranges estimated from GT:
	std::string OUT_DIR;


	// --------------------------------------------------------------------
	//						EXPERIMENT PREPARATION
	// --------------------------------------------------------------------
	CTicTac							tictac,tictacGlobal;
	size_t									rawlogEntry, rawlogEntries;
	CParticleFilter::TParticleFilterStats	PF_stats;

	// Load the set of metric maps to consider in the experiments:
	CMultiMetricMap							metricMap;
	TSetOfMetricMapInitializers				mapList;
	mapList.loadFromConfigFile( *iniFile,"MetricMap");
	metricMap.setListOfMaps( &mapList );
	mapList.dumpToConsole();
	pfOptions.dumpToConsole();

	// Init random & serialization:
	randomGenerator.randomize();


/*	if( MAP_FILE!="NOT FOUND!" )
	{
		// Load map
		// -----------------------------
		CSimpleMap					simpleMap;
		printf("Loading '.simplemap' file...");
		CFileStream(MAP_FILE.c_str(),fomRead) >> simpleMap;
		printf("Ok\n");

		// Build metric map:
		// ------------------------------
		printf("Building metric map from '.simplemap'...");
		metricMap.loadFromProbabilisticPosesAndObservations(simpleMap);
		printf("Ok\n");

		ASSERT_(metricMap.m_gridMaps.size()>0);
	}
*/

	// --------------------------------------------------------------------
	//					EXPERIMENT REPETITIONS LOOP
	// --------------------------------------------------------------------

	// --------------------------
	// Load the rawlog:
	// --------------------------
	printf("Loading the rawlog file...");
	CRawlog			rawlog;
	rawlog.loadFromRawLogFile(RAWLOG_FILE);

	rawlogEntries = rawlog.size();
	std::cout<<"rawlog Entries: "<<rawlogEntries<<"\n";
	printf("OK\n");

	// Number of beacons in the fixed, reference map:
	size_t nBeaconsInMap = 0;
	if (metricMap.m_landmarksMap)
		nBeaconsInMap = metricMap.m_landmarksMap->size();

	CMatrixDouble 	franco_matrix(rawlogEntries,3);
	CMatrixDouble	particle_matrix(rawlogEntries,12+nBeaconsInMap);
	CMatrixDouble	real_ranges(rawlogEntries,nBeaconsInMap);
	CMatrixDouble	real_offsets(rawlogEntries,1+nBeaconsInMap);

	CPose3D			sensorPoseOnRobot;
	bool			sensorPoseOnRobot_valid = false;

	tictacGlobal.Tic();
	for (float range_Pc=Pc_range_ini;range_Pc<=Pc_range_end;range_Pc+=Pc_range_step)
	{
		// The experiment directory is:
		OUT_DIR = format("%s_Pc_%.06f",OUT_DIR_PREFIX.c_str(),range_Pc);
		printf("Creating directory: %s\n",OUT_DIR.c_str());
		mrpt::system::createDirectory( OUT_DIR );

		CVectorDouble vector_errs_xy;

		for (int repetition = 0; repetition <NUM_REPS; repetition++)
		{

			unsigned int PARTICLE_COUNT = pfOptions.sampleSize;
			CPosePDFParticlesExtended	pdf(PARTICLE_COUNT);
			// PDF Options:
			// ------------------
			pdf.options.KLD_binSize_PHI		= DEG2RAD(iniFile->read_float("KLD_options","KLD_binSize_PHI_deg",5));
			pdf.options.KLD_binSize_XY		= iniFile->read_float("KLD_options","KLD_binSize_XY",0.2f);
			pdf.options.KLD_delta			= iniFile->read_float("KLD_options","KLD_delta",0.01f);
			pdf.options.KLD_epsilon			= iniFile->read_float("KLD_options","KLD_epsilon",0.02f);
			pdf.options.KLD_maxSampleSize	= iniFile->read_int("KLD_options","KLD_maxSampleSize",100000);
			pdf.options.KLD_minSampleSize	= iniFile->read_int("KLD_options","KLD_minSampleSize",250);

			pdf.options.probabilityChangingBias = iniFile->read_float("RO_MODEL","probabilityChangingBias",0.1f);
			pdf.options.changingBiasUnifRange   = iniFile->read_float("RO_MODEL","changingBiasUnifRange",2.0f);
			pdf.options.mixtureProposalRatio	= iniFile->read_float("RO_MODEL","mixtureProposalRatio",0.10f);

			size_t		real_offsets_rows=0;
			double accum_xy_err_sqr = 0;
			if (Pc_range_ini!=Pc_range_end)
			{
				pdf.options.probabilityChangingBias = range_Pc;
			}

			pdf.options.metricMap = &metricMap;

			//Create Directory
			OUT_DIR= format("%s_Pc_%.06f/REP_%03u",OUT_DIR_PREFIX.c_str(),range_Pc, (unsigned int)repetition );
			printf("Creating directory: %s\n",OUT_DIR.c_str());

			mrpt::system::deleteFilesInDirectory(OUT_DIR);
			mrpt::system::createDirectory( OUT_DIR );

			// Save the landmarks for plot in matlab:
			if (metricMap.m_landmarksMap)
			{
				metricMap.m_landmarksMap->saveToMATLABScript2D( format("%s/plot_landmarks_map.m",OUT_DIR.c_str()) );
			}

			CPose2D odometryEstimation(
				iniFile->read_float("OdometryEstimation","odometry_X",0),
				iniFile->read_float("OdometryEstimation","odometry_Y",0),
				DEG2RAD(iniFile->read_float("OdometryEstimation","odometry_PHI",0)) );

			CPose2D	initialPoseExperiment( odometryEstimation );

			CParticleFilter PF;
			PF.m_options = pfOptions;

			int step = 0;
			rawlogEntry = 0;

			// Initialize the PDF:
			// -----------------------------
			tictac.Tic();
			CVectorFloat		state_min(nBeaconsInMap, 0.0f);
			CVectorFloat		state_max(nBeaconsInMap, 0.0f);

			pdf.resetUniform(
				initialPoseExperiment.x()-0.5, initialPoseExperiment.x()+0.5,
				initialPoseExperiment.y()-0.5, initialPoseExperiment.y()+0.5,
				state_min, state_max,
				-M_PIf, M_PIf,
				PARTICLE_COUNT );

			printf("PDF of %u particles initialized in %.03fms\n", PARTICLE_COUNT, 1000*tictac.Tac());

			// 3D World
#ifdef STORE_3D
			COpenGLScenePtr scene = COpenGLScene::Create();

	#ifdef	SHOW_REAL_TIME_3D
			CDisplayWindow3D		window("ro-localization - Part of MRPT");
			window.setPos(50,50);
			window.resize(800,500);
			window.setCameraElevationDeg( 90 );
			window.setCameraZoom( 11 );
			window.setCameraAzimuthDeg(-90);
			window.setCameraPointingToPoint(6.0,3.0,0);

			window.setCameraProjective(false);

			if ( SAVE_3D_TO_VIDEO )
			{
				window.grabImagesStart();
			}

			COpenGLScenePtr sceneTR = window.get3DSceneAndLock();
	#endif
			//World Axis
			{
				opengl::CAxisPtr obj = opengl::CAxis::Create(-20,-10,-1,20,10,4,1);
				obj->enableTickMarks();
				obj->setColor(0,0,0);
		#ifdef	SHOW_REAL_TIME_3D
				sceneTR->insert( obj );
		#endif
				scene->insert( obj );
			}


			//Grid2D
			if ( mrpt::system::fileExists(MAP_FILE) )
			{
				COccupancyGridMap2D	grid2d;
				CFileGZInputStream(MAP_FILE) >> grid2d;

	#ifdef	SHOW_REAL_TIME_3D
				{
					opengl::CSetOfObjectsPtr obj = opengl::CSetOfObjects::Create();
					grid2d.getAs3DObject(obj);
					obj->setLocation(initialPoseExperiment.x(),initialPoseExperiment.y(), 0);
					sceneTR->insert( obj );
				}
	#endif
				opengl::CSetOfObjectsPtr obj = opengl::CSetOfObjects::Create();
				grid2d.getAs3DObject(obj);
				scene->insert( obj );
			}
			else
			// Floor
			{
				opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-20,20,-10,10,0,0.5);
				obj->setColor(0.4,0.4,0.4);
		#ifdef	SHOW_REAL_TIME_3D
				sceneTR->insert( obj );
		#endif
				scene->insert( obj );
			}


			// The beacons
			if (metricMap.m_landmarksMap)
			{
				for (size_t k=0;k<metricMap.m_landmarksMap->size();k++)
				{
					opengl::CSpherePtr parts = opengl::CSphere::Create();
					parts->setColor(1,0,0);
					parts->setLocation( metricMap.m_landmarksMap->landmarks.get(k)->pose_mean );
					parts->setRadius(0.2f);

					parts->setName(format(" B%i",int(k+1)));
					parts->enableShowName();
					scene->insert( parts );
			#ifdef	SHOW_REAL_TIME_3D
					sceneTR->insert( parts );
			#endif
				}
			}


	#ifdef	SHOW_REAL_TIME_3D
			window.unlockAccess3DScene();
			window.forceRepaint();
	#endif
#endif
			// -----------------------------
			//		Particle filter
			// -----------------------------
			CActionCollectionPtr action;
			CSensoryFramePtr observations;
			CPose2D	pdfEstimation;

			while (rawlogEntry<(rawlogEntries-1))
			{
				// Load pose change from the rawlog:
				// ----------------------------------------
				cout << endl << "RAWLOG_ENTRY: " << rawlogEntry << endl << endl;

				if (!rawlog.getActionObservationPair(action,observations,rawlogEntry))
					break; // end of rawlog.

				//-------------------FRANCO DATA--------------------
				{
					CObservationBeaconRangesPtr beaconPose = observations->getObservationByClass<CObservationBeaconRanges>();
					if (beaconPose)
					{
						franco_matrix(step,0)=beaconPose->auxEstimatePose.x();
						franco_matrix(step,1)=beaconPose->auxEstimatePose.y();
						if (beaconPose->auxEstimatePose.phi()>180)
							franco_matrix(step,2)=1e10;
						else if(beaconPose->auxEstimatePose.phi()<-180)
							franco_matrix(step,2)=-1e10;
						else
							franco_matrix(step,2)=beaconPose->auxEstimatePose.phi();
					}
					else
					{
						franco_matrix(step,0)=-6;
						franco_matrix(step,1)=-6;
						franco_matrix(step,1)=-6;
					}
				}
				//--------------------------------------------------
				// PARTICLE FILTER:
				// ----------------------------------------
				tictac.Tic();
				printf("Executing ParticleFilter on %u particles....", (unsigned int)pdf.particlesCount());

				PF.executeOn(
						pdf,
						action.pointer(),			// Action
						observations.pointer(),	// Obs.
						&PF_stats		// Output statistics
						);
				printf(" Done! in %.03fms, ESS=%f\n", 1000.0f*tictac.Tac(), pdf.ESS());

				// Text output:
				// ----------------------------------------
				odometryEstimation = odometryEstimation + action->getBestMovementEstimation()->poseChange->getMeanVal();
				pdf.getMean( pdfEstimation );

				CPose2D  GT_Pose;
				if (groundTruth.getRowCount()>size_t(step))
				{
					GT_Pose = CPose2D( groundTruth(step,1),groundTruth(step,2), groundTruth(step,3) );
					GT_Pose = initialPoseExperiment + GT_Pose;

					// Add error:
					accum_xy_err_sqr += GT_Pose.sqrDistanceTo( pdfEstimation );

					// Create real_ranges row:
					// ----------------------------
					CObservationBeaconRangesPtr beaconPose = observations->getObservationByClass<CObservationBeaconRanges>();
					if (!sensorPoseOnRobot_valid)
					{
						if (beaconPose)
						{
							beaconPose->getSensorPose( sensorPoseOnRobot );
							sensorPoseOnRobot_valid = true;
						}
					}

					if (sensorPoseOnRobot_valid)
					{
						CPose3D  sensorPose = GT_Pose + sensorPoseOnRobot;
						bool any_valid=false;
						for (size_t q=0;q<nBeaconsInMap;q++)
						{
							mrpt::maps::CLandmark *lm = metricMap.m_landmarksMap->landmarks.get(q);
							// Real range
							double R_real = sensorPose.distanceTo( lm->pose_mean );
							real_ranges(step,q) = R_real;

							// Real offset:
							if (beaconPose)
							{
								float R = beaconPose->getSensedRangeByBeaconID(q+1);
								if (R>0)
								{
									any_valid=true;
									real_offsets(real_offsets_rows,1+q) = R - R_real;
								}
							}
						}
						if (any_valid)
						{
							real_offsets(real_offsets_rows,0) = step;
							real_offsets_rows++;
						}
					}

				} // end GT

				TExtendedCPose2D	meanState = pdf.getEstimatedPoseState();

				//likelihood_acumulation += pdfEstimation.distanceTo(CPoint2D (groundTruth(step,1),groundTruth(step,2)));

				std::cout << "    Odometry est: " << odometryEstimation << "\n";
				std::cout << "         PDF est: " << pdfEstimation << ", ESS (B.R.)= " << PF_stats.ESS_beforeResample << " Cov:\n";

				CMatrixDouble	cov;
				pdf.getCovariance(cov);

				std::cout << cov << "\n sqrt(Det.Cov) = " << sqrt(cov.det()) <<"\n";

				//-----------------PARTICLE DATA-----------------
				particle_matrix(step,0)=pdfEstimation.x();
				particle_matrix(step,1)=pdfEstimation.y();
				particle_matrix(step,2)=pdfEstimation.phi();

				particle_matrix(step,3)= cov(0,0);
				particle_matrix(step,4)= cov(1,1);
				particle_matrix(step,5)= cov(2,2);
				particle_matrix(step,6)= cov(0,1);
				particle_matrix(step,7)= cov(0,2);
				particle_matrix(step,8)= cov(1,2);

				particle_matrix(step,9)=GT_Pose.x();
				particle_matrix(step,10)=GT_Pose.y();
				particle_matrix(step,11)=GT_Pose.phi();

				ASSERT_EQUAL_( size_t(meanState.state.size()), size_t(nBeaconsInMap) );
				for (size_t l=0;l<nBeaconsInMap;l++)
					particle_matrix(step,12+l)= meanState.state[l];
				//----------------------------------------------


				//// Generate text files for matlab:
				//// ------------------------------------
				//pdf.saveToTextFile(format("%s/particles_%03u.txt",OUT_DIR.c_str(),step));

				/*{
					CObservation2DRangeScan		*o = (CObservation2DRangeScan*) observations->getObservationByClass( CLASS_ID(CObservation2DRangeScan));
					if (o)
					{
						MRPT_OS::sprintf(auxDirStr,1000,"%s/observation_scan_%03u.txt",OUT_DIR,step);
						FILE	*f=MRPT_OS::fopen(auxDirStr,"wt");
						for (unsigned int i=0;i<o->scan.size();i++)
							fprintf(f,"%.03f ",o->scan[i]);
						fclose(f);
					}
				}*/

#ifdef STORE_3D
				// Generate 3D scene:
				// ------------------------------
				{
					CPose2D			meanPose;
					CMatrixDouble33	cov;
					pdf.getCovarianceAndMean(cov,meanPose);

					//// The gridmap:
					//if (metricMap.m_gridMaps.size())
					//{
					//	opengl::CSetOfObjects	*plane = new opengl::CSetOfObjects();
					//	metricMap.m_gridMaps[0]->getAs3DObject( *plane );
					//	scene->insert( plane );
					//}

	#ifdef	SHOW_REAL_TIME_3D
					sceneTR = window.get3DSceneAndLock();
	#endif


					// The particles:
	#ifdef	SHOW_REAL_TIME_3D
					opengl::CRenderizablePtr obj = sceneTR->getByName( "part" );
					opengl::CPointCloudPtr parts;

					if( !obj.present() )
						parts = opengl::CPointCloud::Create();
					else
						parts = CPointCloudPtr(obj);
	#else
					opengl::CPointCloudPtr parts = opengl::CPointCloud::Create();
					opengl::CRenderizablePtr obj;
	#endif

					parts->setColor(0,0,1);
					parts->setName( "part");
					parts->enableColorFromZ(false);

					parts->resize( pdf.size() );

					for (size_t i=0;i<pdf.size();i++)
						parts->setPoint(i,pdf.m_particles[i].d->pose.x(),pdf.m_particles[i].d->pose.y(), 0);

					if( !obj.present() )
					{
						scene->insert( parts );
	#ifdef	SHOW_REAL_TIME_3D
						sceneTR->insert( parts );
	#endif
					}


					// The particles' cov:
	#ifdef	SHOW_REAL_TIME_3D
					obj = sceneTR->getByName( "cov" );
					opengl::CEllipsoidPtr ellip;
					if( !obj.present() )
						ellip = opengl::CEllipsoid::Create();
					else
						ellip = CEllipsoidPtr(obj);
	#else
					opengl::CEllipsoidPtr ellip =  opengl::CEllipsoid::Create();
	#endif

					ellip->setColor(1,0,0, 0.6);
					ellip->setLocation( meanPose.x(), meanPose.y(), 0.05);

					ellip->setLineWidth(2);
					ellip->setQuantiles(3);
					ellip->setCovMatrix(cov, 2);
					ellip->setName("cov");

					if( !obj.present() )
					{
						scene->insert( ellip );
	#ifdef	SHOW_REAL_TIME_3D
						sceneTR->insert( ellip );
	#endif
					}

	#ifdef	SHOW_REAL_TIME_3D
					// The laser scan:
					obj = sceneTR->getByName( "laser" );
					opengl::CPointCloudPtr scanPts;
					if( !obj.present() )
						scanPts = opengl::CPointCloud::Create();
					else
						scanPts = CPointCloudPtr(obj);

					scanPts->setColor(1,0,0, 0.9);
					scanPts->enableColorFromZ(false);
					scanPts->setPointSize(3);
					scanPts->setName("laser");

					CSimplePointsMap	map;
					CPose3D				robotPose3D( meanPose );
					observations->insertObservationsInto( &map, &robotPose3D );
					scanPts->loadFromPointsMap( &map );

					if( !obj.present() )
					{
						scene->insert( scanPts );
						sceneTR->insert( scanPts );
					}

					// Beacon range spheres:
					CObservationBeaconRangesPtr dist=  observations->getObservationByClass<CObservationBeaconRanges>();
					if ( metricMap.m_landmarksMap && dist.present() && !dist->sensedData.empty() )
					{
						for (size_t k=0;k<dist->sensedData.size();k++)
						{
							string beacon_name = format("ring%u",unsigned(dist->sensedData[k].beaconID));
							const mrpt::maps::CLandmark *lm = metricMap.m_landmarksMap->landmarks.getByBeaconID( dist->sensedData[k].beaconID );
							if (lm)
							{
								#ifdef	SHOW_REAL_TIME_3D
									opengl::CRenderizablePtr obj = sceneTR->getByName( beacon_name );
									opengl::CDiskPtr sphere;

									if( !obj.present() )
										sphere = opengl::CDisk::Create();
									else
										sphere = CDiskPtr(obj);
								#else
									opengl::CSpherePtr sphere = opengl::CSphere::Create();
									opengl::CRenderizablePtr obj;
								#endif

								sphere->setColor(0,0,1, 0.3);
								sphere->setLoopsCount(10);
								sphere->setSlicesCount(40);
								sphere->setName(beacon_name);

								sphere->setLocation(lm->pose_mean.x,lm->pose_mean.y,0.05 );

								float R =
									square(dist->sensedData[k].sensedDistance) -
									square(dist->sensedData[k].sensorLocationOnRobot.z() - lm->pose_mean.z);

								if (R>0) R=sqrt(R); else R=0.08f;
								sphere->setDiskRadius(R+0.08f, R-0.08f);

								if( !obj.present() )
								{
									scene->insert( sphere );

									#ifdef	SHOW_REAL_TIME_3D
										sceneTR->insert( sphere );
									#endif
								}
							}
						}
					} // for each beacon

					// Franco Position
					if (SHOW_3D_FRANCO_POSITION)
					{
						opengl::CRenderizablePtr obj = sceneTR->getByName( "franc" );
						opengl::CSpherePtr sphere;

						if( !obj.present() )
							sphere = opengl::CSphere::Create();
						else
							sphere = opengl::CSpherePtr(obj);
						sphere->setColor(0,1,0);
						sphere->setRadius(0.05f);
						sphere->setName( "franc");

						CObservationBeaconRangesPtr dist= observations->getObservationByClass<CObservationBeaconRanges>();
						if (dist.present())
						{
							sphere->setLocation(
								dist->auxEstimatePose.x(),
								dist->auxEstimatePose.y(),
								0.05f );
						}

						if( !obj.present() )
						{
							scene->insert( sphere );
							sceneTR->insert( sphere );
						}
					}
	#endif

					// Mean of particles
					{
	#ifdef	SHOW_REAL_TIME_3D
						opengl::CRenderizablePtr obj = sceneTR->getByName( "mean_parts" );
						opengl::CSpherePtr sphere;

						if( !obj.present() )
							sphere = opengl::CSphere::Create();
						else
							sphere = opengl::CSpherePtr(obj);

	#else
						opengl::CSpherePtr sphere = opengl::CSphere::Create();
	#endif
						sphere->setColor( 0,0,1);
						sphere->setRadius(0.05f);
						sphere->setName( "mean_parts" );

						sphere->setLocation( pdfEstimation.x(), pdfEstimation.y(), 0.05);

						if( !obj.present() )
						{
							scene->insert( sphere );
	#ifdef	SHOW_REAL_TIME_3D
							sceneTR->insert( sphere );
	#endif
						}
					}

					// groundTruth
					if (groundTruth.getRowCount()>size_t(step))
					{
	#ifdef	SHOW_REAL_TIME_3D
						opengl::CRenderizablePtr obj = sceneTR->getByName( "GT" );
						opengl::CSpherePtr sphere;

						if( !obj.present() )
							sphere = opengl::CSphere::Create();
						else
							sphere = opengl::CSpherePtr(obj);
	#else
						opengl::CSpherePtr sphere = opengl::CSphere::Create();
	#endif
						sphere->setColor(0,0,0);
						sphere->setRadius(0.10f);
						sphere->setName("GT");

						sphere->setLocation(GT_Pose.x(), GT_Pose.y(), 0.05);

						cout << "GT robot pose: " << GT_Pose << endl;


						if( !obj.present() )
						{
							scene->insert( sphere );
	#ifdef	SHOW_REAL_TIME_3D
							sceneTR->insert( sphere );
	#endif
						}
					}

	#ifdef	SHOW_REAL_TIME_3D
					//GPS pose
					{
						CObservationGPSPtr o = observations->getObservationByClass<CObservationGPS>();
						if (o.present() && metricMap.m_landmarksMap)
						{
							opengl::CRenderizablePtr obj = sceneTR->getByName( "gps" );
							opengl::CEllipsoidPtr sphere;
							double x,y;

							if( !obj.present() )
								sphere = opengl::CEllipsoid::Create();
							else
								sphere = opengl::CEllipsoidPtr(obj);

							sphere->setColor(0,1,1, 0.5);
							sphere->setName( "gps");
							if (o->hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>())
							{
								const mrpt::obs::gnss::Message_NMEA_GGA &gga = o->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
								x = DEG2RAD((gga.fields.longitude_degrees-metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.longitude))*6371000*1.03;
								y = DEG2RAD((gga.fields.latitude_degrees-metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.latitude))*6371000*1.15;
								sphere->setLocation(
									(x*cos(metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.ang)+y*sin(metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.ang)+metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.x_shift),
									(-x*sin(metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.ang)+y*cos(metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.ang)+metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.y_shift),
									0 );
							}
							CMatrix	r(2,2);
							r(1,1)=9;
							r(0,0)=9;
							sphere->setCovMatrix(r);
							if( !obj.present() )
							{
								scene->insert( sphere );
								sceneTR->insert( sphere );
							}
						}
					}
					{
						CObservationGPSPtr o = observations->getObservationByClass<CObservationGPS>();
						if (o.present())
						{
							opengl::CRenderizablePtr obj = sceneTR->getByName( "gps_CENTER" );
							opengl::CSpherePtr sphere;
							double x,y;

							if( !obj.present() )
								sphere = opengl::CSphere::Create();
							else
								sphere = opengl::CSpherePtr(obj);
							sphere->setColor(0,1,1);
							sphere->setName("gps_CENTER");
							if (o->hasMsgClass<mrpt::obs::gnss::Message_NMEA_GGA>())
							{
								const mrpt::obs::gnss::Message_NMEA_GGA &gga = o->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
								x = DEG2RAD((gga.fields.longitude_degrees-metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.longitude))*6371000*1.03;//*9000000;
								y = DEG2RAD((gga.fields.latitude_degrees-metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.latitude))*6371000*1.15;//*12000000;
								sphere->setLocation(
									(x*cos(metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.ang)+y*sin(metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.ang)+metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.x_shift),
									(-x*sin(metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.ang)+y*cos(metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.ang)+metricMap.m_landmarksMap->likelihoodOptions.GPSOrigin.y_shift),
									0);
							}
							sphere->setRadius(0.5);
							if( !obj.present() )
							{
								scene->insert( sphere );
								sceneTR->insert( sphere );
							}
						}
					}

					window.unlockAccess3DScene();
					window.forceRepaint();
					mrpt::system::sleep(2);	// Time to refresh
	#endif
				}
#endif

				if ((step % SCENE3D_FREQ)==0)
					CFileGZOutputStream( format("%s/3Dscene_%03u.3Dscene",OUT_DIR.c_str(),step) ) << *scene;

				step++;
			}// while rawlogEntries
			//aux(0,0)=(float)likelihood_acumulation;
			//aux.saveToTextFile( format("%s/LIKELIHOOD.txt",OUT_DIR.c_str()) );

			franco_matrix.setSize(step,3);
			franco_matrix.saveToTextFile(format("%s/franco_matrix.txt",OUT_DIR.c_str()));

			particle_matrix.setSize(step, particle_matrix.getColCount() );
			particle_matrix.saveToTextFile(format("%s/particle_matrix.txt",OUT_DIR.c_str()), MATRIX_FORMAT_FIXED);

			real_ranges.setSize(step, real_ranges.getColCount() );
			real_ranges.saveToTextFile(format("%s/GT_ranges.txt",OUT_DIR.c_str()), MATRIX_FORMAT_FIXED);

			real_offsets.setSize(real_offsets_rows, real_offsets.getColCount() );
			real_offsets.saveToTextFile(format("%s/GT_offsets.txt",OUT_DIR.c_str()), MATRIX_FORMAT_FIXED);

			// Error stats:
			if (step>0)
			{
				double avrg_xy_err = sqrt( accum_xy_err_sqr / step );
				vector_errs_xy.push_back( avrg_xy_err );
			}


		}	// repetitions

		// Average errors:
		if (Pc_range_ini!=Pc_range_end)
		{
			CFileStream fo( format( "Pc_%.06f_%.06f_RESULTs.txt", Pc_range_ini, Pc_range_end ), fomAppend );
			double err_mean, err_std;
			mrpt::math::meanAndStd( vector_errs_xy, err_mean, err_std);
			fo.printf("%f %f %f\n", range_Pc, err_mean, err_std);
		}

	}	//for Pc



	printf("\n TOTAL EXECUTION TIME = %.06f sec\n\n", tictacGlobal.Tac() );

	//mrpt::system::pause();
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" ro-localization - Version 0.1 - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2)
		{
			printf("Usage: %s <config.ini>\n\n",argv[0]);
			//pause();
			return -1;
		}

		iniFileName = argv[1];

		iniFile = new CConfigFile(iniFileName);

		TestParticlesLocalization();

		delete iniFile;

		return 0;
	}
	catch (exception &e)
	{
		cout << "Caught MRPT exception:\n" << e.what() << endl;
		//pause();
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		//pause();
		return -1;
	}
}
