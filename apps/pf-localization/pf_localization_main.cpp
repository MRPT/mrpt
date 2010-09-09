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


/*---------------------------------------------------------------
	APPLICATION: Particle Filter (Global) Localization Demo
	FILE: pf_localization_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <jlblanco@ctima.uma.es>

	See README.txt for instructions.
  ---------------------------------------------------------------*/

#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace std;

// Forward declaration:
void TestParticlesLocalization(const std::string &iniFilename);
void getGroundTruth( CPose2D &expectedPose, size_t rawlogEntry, const CMatrixDouble &GT, const TTimeStamp &cur_time);


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" ParticleFilter Localization\n");
		printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2)
		{
			printf("Usage: %s <config.ini>\n\n",argv[0]);
			return -1;
		}

		TestParticlesLocalization( argv[1] );

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


// ------------------------------------------------------
//				TestParticlesLocalization
// ------------------------------------------------------
void TestParticlesLocalization(const std::string &ini_fil)
{
	ASSERT_( fileExists(ini_fil) );

	CConfigFile	iniFile(ini_fil);

	vector_int			particles_count;	// Number of initial particles (if size>1, run the experiments N times)

	// Load configuration:
	// -----------------------------------------
	string iniSectionName ( "LocalizationExperiment" );


	// Mandatory entries:
	iniFile.read_vector(iniSectionName, "particles_count", vector_int(1,0), particles_count, /*Fail if not found*/true );
	string		RAWLOG_FILE			= iniFile.read_string(iniSectionName,"rawlog_file","", /*Fail if not found*/true );
	string		OUT_DIR_PREFIX		= iniFile.read_string(iniSectionName,"logOutput_dir","", /*Fail if not found*/true );

	// Non-mandatory entries:
	string		MAP_FILE			= iniFile.read_string(iniSectionName,"map_file","" );
	size_t		rawlog_offset		= iniFile.read_int(iniSectionName,"rawlog_offset",0);
	string		GT_FILE				= iniFile.read_string(iniSectionName,"ground_truth_path_file","");
	int		NUM_REPS			= iniFile.read_int(iniSectionName,"experimentRepetitions",1);
	int		SCENE3D_FREQ		= iniFile.read_int(iniSectionName,"3DSceneFrequency",10);
	bool 		SCENE3D_FOLLOW = iniFile.read_bool(iniSectionName,"3DSceneFollowRobot",true);
	unsigned int	testConvergenceAt   = iniFile.read_int(iniSectionName,"experimentTestConvergenceAtStep",-1);

	bool    	SAVE_STATS_ONLY = iniFile.read_bool(iniSectionName,"SAVE_STATS_ONLY",false);

	bool 		SHOW_PROGRESS_3D_REAL_TIME = iniFile.read_bool(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME",false);
	int			SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = iniFile.read_int(iniSectionName,"SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS",1);
	double 		STATS_CONF_INTERVAL = iniFile.read_double(iniSectionName,"STATS_CONF_INTERVAL",0.2);

	// PF-algorithm Options:
	// ---------------------------
	CParticleFilter::TParticleFilterOptions		pfOptions;
	pfOptions.loadFromConfigFile( iniFile, "PF_options" );

	// PDF Options:
	// ------------------
	TMonteCarloLocalizationParams	pdfPredictionOptions;
	pdfPredictionOptions.KLD_params.loadFromConfigFile( iniFile, "KLD_options");

	// Metric map options:
	// -----------------------------
	TSetOfMetricMapInitializers				mapList;
	mapList.loadFromConfigFile( iniFile,"MetricMap");



	printf("\n-------------------------------------------------------------\n");
	printf("\t RAWLOG_FILE = \t %s\n",RAWLOG_FILE.c_str());
	printf("\t MAP_FILE = \t %s\n",MAP_FILE.c_str());
	printf("\t GT_FILE = \t %s\n",GT_FILE.c_str());
	printf("\t OUT_DIR_PREFIX = \t %s\n",OUT_DIR_PREFIX.c_str());
	printf("\t #particles = \t "); cout << particles_count << "\n";
	printf("-------------------------------------------------------------\n");
	pfOptions.dumpToConsole();
	mapList.dumpToConsole();

	// --------------------------------------------------------------------
	//						EXPERIMENT PREPARATION
	// --------------------------------------------------------------------
	CTicTac		tictac,tictacGlobal;
	CSimpleMap	simpleMap;
	CRawlog		rawlog;
	size_t		rawlogEntry, rawlogEntries;
	CParticleFilter::TParticleFilterStats	PF_stats;

	// Load the set of metric maps to consider in the experiments:
	CMultiMetricMap							metricMap;
	metricMap.setListOfMaps( &mapList );
	mapList.dumpToConsole();

	randomGenerator.randomize();

	// Load the map (if any):
	// -------------------------
	if (MAP_FILE.size())
	{
		ASSERT_( fileExists(MAP_FILE) );

		// Detect file extension:
		// -----------------------------
		string mapExt = lowerCase( extractFileExtension( MAP_FILE, true ) ); // Ignore possible .gz extensions

		if ( !mapExt.compare( "simplemap" ) )
		{
			// It's a ".simplemap":
			// -------------------------
			printf("Loading '.simplemap' file...");
			CFileGZInputStream(MAP_FILE.c_str()) >> simpleMap;
			printf("Ok\n");

			ASSERT_( simpleMap.size()>0 );

			// Build metric map:
			// ------------------------------
			printf("Building metric map(s) from '.simplemap'...");
			metricMap.loadFromProbabilisticPosesAndObservations(simpleMap);
			printf("Ok\n");
		}
		else if ( !mapExt.compare( "gridmap" ) )
		{
			// It's a ".gridmap":
			// -------------------------
			printf("Loading gridmap from '.gridmap'...");
			ASSERT_( metricMap.m_gridMaps.size()==1 );
			CFileGZInputStream(MAP_FILE) >> (*metricMap.m_gridMaps[0]);
			printf("Ok\n");
		}
		else
		{
			THROW_EXCEPTION_CUSTOM_MSG1("Map file has unknown extension: '%s'",mapExt.c_str());
		}

	}

	// --------------------------
	// Load the rawlog:
	// --------------------------
	printf("Opening the rawlog file...");
	rawlog.loadFromRawLogFile(RAWLOG_FILE);
	rawlogEntries = rawlog.size();
	printf("OK\n");


	// Load the Ground Truth:
	CMatrixDouble	GT(0,0);
	if ( fileExists( GT_FILE ) )
	{
		printf("Loading ground truth file...");
		GT.loadFromTextFile( GT_FILE );
		printf("OK\n");
	}
	else
		printf("Ground truth file: NO\n");


	// The experiment directory is:
//	string sOUT_DIR_OVERALL = format("%s_SUMMARY",OUT_DIR_PREFIX.c_str());
//	const char * OUT_DIR_OVERALL = sOUT_DIR_OVERALL.c_str();
//	createDirectory( OUT_DIR_OVERALL );
//	deleteFiles( format("%s/*.*",OUT_DIR_OVERALL));


	// Create 3D window if requested:
	CDisplayWindow3DPtr	win3D;
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = CDisplayWindow3D::Create("PF localization @ MRPT C++ Library (C) 2004-2008", 1000, 600);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);

	}

	// Create the 3D scene and get the map only once, later we'll modify only the particles, etc..
	COpenGLScene			scene;
	COccupancyGridMap2D::TEntropyInfo	gridInfo;

	// The gridmap:
	if (metricMap.m_gridMaps.size())
	{
		metricMap.m_gridMaps[0]->computeEntropy( gridInfo );
		printf("The gridmap has %.04fm2 observed area, %u observed cells\n", gridInfo.effectiveMappedArea, (unsigned) gridInfo.effectiveMappedCells );

		{
			CSetOfObjectsPtr plane = CSetOfObjects::Create();
			metricMap.m_gridMaps[0]->getAs3DObject( plane );
			scene.insert( plane );
		}

		if (SHOW_PROGRESS_3D_REAL_TIME)
		{
			COpenGLScenePtr ptrScene = win3D->get3DSceneAndLock();

			CSetOfObjectsPtr plane = CSetOfObjects::Create();
			metricMap.m_gridMaps[0]->getAs3DObject( plane );
			ptrScene->insert( plane );

			ptrScene->enableFollowCamera(true);

			win3D->unlockAccess3DScene();
		}
	}


	for ( vector_int::iterator itNum = particles_count.begin(); itNum!=particles_count.end(); ++itNum )
	{
		int		PARTICLE_COUNT = *itNum;

		printf("Initial PDF: %f particles/m2\n", PARTICLE_COUNT/gridInfo.effectiveMappedArea);


		// Global stats for all the experiment loops:
		int				nConvergenceTests = 0, nConvergenceOK = 0;
		vector_double 	covergenceErrors;
		covergenceErrors.reserve(NUM_REPS);
		// --------------------------------------------------------------------
		//					EXPERIMENT REPETITIONS LOOP
		// --------------------------------------------------------------------
		tictacGlobal.Tic();
		for (int repetition = 0; repetition <NUM_REPS; repetition++)
		{
			printf("\n-------------------------------------------------------------\n");
			printf("      RUNNING FOR %u INITIAL PARTICLES  - Repetition %u / %u\n", PARTICLE_COUNT,1+repetition,NUM_REPS);
			printf("-------------------------------------------------------------\n\n");


			// The experiment directory is:
			const char  *OUT_DIR=NULL;
			const char  *OUT_DIR_PARTS=NULL;
			const char  *OUT_DIR_3D=NULL;
			string      sOUT_DIR;
			string      sOUT_DIR_PARTS;
			string      sOUT_DIR_3D;

			if (!SAVE_STATS_ONLY)
			{
				sOUT_DIR        = format("%s_%03u",OUT_DIR_PREFIX.c_str(),repetition );
				OUT_DIR        = sOUT_DIR.c_str();

				sOUT_DIR_PARTS  = format("%s/particles", OUT_DIR);
				OUT_DIR_PARTS  = sOUT_DIR_PARTS.c_str();

				sOUT_DIR_3D = format("%s/3D", OUT_DIR);
				OUT_DIR_3D  = sOUT_DIR_3D.c_str();

				printf("Creating directory: %s\n",OUT_DIR);
				createDirectory( OUT_DIR );
				ASSERT_(fileExists(OUT_DIR));
				deleteFiles(format("%s/*.*",OUT_DIR));

				printf("Creating directory: %s\n",OUT_DIR_PARTS);
				createDirectory( OUT_DIR_PARTS );
				ASSERT_(fileExists(OUT_DIR_PARTS));
				deleteFiles(format("%s/*.*",OUT_DIR_PARTS));

				printf("Creating directory: %s\n",OUT_DIR_3D);
				createDirectory( OUT_DIR_3D );
				ASSERT_(fileExists(OUT_DIR_3D));
				deleteFiles(format("%s/*.*",OUT_DIR_3D));

				metricMap.m_gridMaps[0]->saveAsBitmapFile(format("%s/gridmap.png",OUT_DIR));
				CFileOutputStream(format("%s/gridmap_limits.txt",OUT_DIR)).printf(
					"%f %f %f %f",
					metricMap.m_gridMaps[0]->getXMin(),metricMap.m_gridMaps[0]->getXMax(),
					metricMap.m_gridMaps[0]->getYMin(),metricMap.m_gridMaps[0]->getYMax() );

				// Save the landmarks for plot in matlab:
				if (metricMap.m_landmarksMap)
					metricMap.m_landmarksMap->saveToMATLABScript2D(format("%s/plot_landmarks_map.m",OUT_DIR));
			}

			int						M = PARTICLE_COUNT;
			CMonteCarloLocalization2D  pdf(M);

			// PDF Options:
			pdf.options = pdfPredictionOptions;

			pdf.options.metricMap = &metricMap;

			// Create the PF object:
			CParticleFilter	PF;
			PF.m_options = pfOptions;

			size_t	step = 0;
			rawlogEntry = 0;

			// Initialize the PDF:
			// -----------------------------
			tictac.Tic();
			if ( !iniFile.read_bool(iniSectionName,"init_PDF_mode",false, /*Fail if not found*/true) )
				pdf.resetUniformFreeSpace(
					metricMap.m_gridMaps[0].pointer(),
					0.7f,
					PARTICLE_COUNT ,
					iniFile.read_float(iniSectionName,"init_PDF_min_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_min_y",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_y",0,true),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_max_phi_deg",180))
					);
			else
				pdf.resetUniform(
					iniFile.read_float(iniSectionName,"init_PDF_min_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_x",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_min_y",0,true),
					iniFile.read_float(iniSectionName,"init_PDF_max_y",0,true),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_min_phi_deg",-180)),
					DEG2RAD(iniFile.read_float(iniSectionName,"init_PDF_max_phi_deg",180)),
					PARTICLE_COUNT
					);


			printf("PDF of %u particles initialized in %.03fms\n", M, 1000*tictac.Tac());

			// -----------------------------
			//		Particle filter
			// -----------------------------
			CActionCollectionPtr action;
			CSensoryFramePtr     observations;
			CPose2D				pdfEstimation, odometryEstimation;
			CMatrixDouble		cov;
			bool				end = false;

			CFileOutputStream   f_cov_est,f_pf_stats,f_odo_est;

			if (!SAVE_STATS_ONLY)
			{
				f_cov_est.open(OUT_DIR+string("/cov_est.txt"));
				f_pf_stats.open(OUT_DIR+string("/PF_stats.txt"));
				f_odo_est.open(OUT_DIR+string("/odo_est.txt"));
			}

			TTimeStamp cur_obs_timestamp;

			while (rawlogEntry<(rawlogEntries-1) && !end)
			{
				// Finish if ESC is pushed:
				if (os::kbhit())
					if (os::getch()==27)
						end = true;

				// Load pose change from the rawlog:
				// ----------------------------------------
				if (!rawlog.getActionObservationPair(action, observations, rawlogEntry ))
					THROW_EXCEPTION("End of rawlog");

				CPose2D		expectedPose; // Ground truth

				if (observations->size()>0)
					cur_obs_timestamp = observations->getObservationByIndex(0)->timestamp;

				if (step>=rawlog_offset)
				{
					// Do not execute the PF at "step=0", to let the initial PDF to be
					//   reflected in the logs.
					if (step>rawlog_offset)
					{
						// Show 3D?
						if (SHOW_PROGRESS_3D_REAL_TIME)
						{
							CPose2D       meanPose;
							CMatrixDouble33 cov;
							pdf.getCovarianceAndMean(cov,meanPose);

							if (rawlogEntry>=2)
								getGroundTruth(expectedPose, rawlogEntry-2, GT, cur_obs_timestamp );

							COpenGLScenePtr ptrScene = win3D->get3DSceneAndLock();

							win3D->setCameraPointingToPoint(meanPose.x(),meanPose.y(),0);

							// The Ground Truth (GT):
							{
								CRenderizablePtr GTpt = ptrScene->getByName("GT");
								if (!GTpt)
								{
									GTpt = CDisk::Create();
									GTpt->setName( "GT" );
									GTpt->setColor(0,0,0, 0.9);

									getAs<CDisk>(GTpt)->setDiskRadius(0.04);
									ptrScene->insert( GTpt );
								}

								GTpt->setPose( expectedPose );
							}


							// The particles:
							{
								CRenderizablePtr parts = ptrScene->getByName("particles");
								if (parts) ptrScene->removeObject(parts);

								CSetOfObjectsPtr p = pdf.getAs3DObject<CSetOfObjectsPtr>();
								p->setName("particles");
								ptrScene->insert(p);
							}

							// The particles' cov:
							{
								CRenderizablePtr	ellip = ptrScene->getByName("parts_cov");
								if (!ellip)
								{
									ellip = CEllipsoid::Create();
									ellip->setName( "parts_cov");
									ellip->setColor(1,0,0, 0.6);

									getAs<CEllipsoid>(ellip)->setLineWidth(2);
									getAs<CEllipsoid>(ellip)->setQuantiles(3);
									ptrScene->insert( ellip );
								}
								ellip->setLocation(meanPose.x(), meanPose.y(), 0.05 );

								getAs<CEllipsoid>(ellip)->setCovMatrix(cov,2);
							}


							// The laser scan:
							{
								CRenderizablePtr scanPts = ptrScene->getByName("scan");
								if (!scanPts)
								{
									scanPts = CPointCloud::Create();
									scanPts->setName( "scan" );
									scanPts->setColor(1,0,0, 0.9);
									getAs<CPointCloud>(scanPts)->enableColorFromZ(false);
									getAs<CPointCloud>(scanPts)->setPointSize(4);
									ptrScene->insert(scanPts);
								}

								CSimplePointsMap	map;
								static CSimplePointsMap	last_map;

								CPose3D				robotPose3D( meanPose );

								map.clear();
								observations->insertObservationsInto( &map );

								getAs<CPointCloud>(scanPts)->loadFromPointsMap( &last_map );
								getAs<CPointCloud>(scanPts)->setPose( robotPose3D );
								last_map = map;
							}

							// The camera:
							ptrScene->enableFollowCamera(true);

							// Views:
							COpenGLViewportPtr view1= ptrScene->getViewport("main");
							{
								CCamera  &cam = view1->getCamera();
								cam.setAzimuthDegrees(-90);
								cam.setElevationDegrees(90);
								cam.setPointingAt( meanPose );
								cam.setZoomDistance(5);
								cam.setOrthogonal();
							}

							COpenGLViewportPtr view2= ptrScene->createViewport("small_view"); // Create, or get existing one.
							view2->setCloneView("main");
							view2->setCloneCamera(false);
							view2->setBorderSize(3);
							{
								CCamera  &cam = view1->getCamera();
								cam.setAzimuthDegrees(-90);
								cam.setElevationDegrees(90);
								cam.setPointingAt( meanPose );
								cam.setZoomDistance(15);
								cam.setOrthogonal();

								view2->setTransparent(false);
								view2->setViewportPosition(0.59,0.01,0.4,0.3);
							}

							win3D->unlockAccess3DScene();

							// Move camera:
							//win3D->setCameraPointingToPoint( curRobotPose.x, curRobotPose.y, curRobotPose.z );

							// Update:
							win3D->forceRepaint();

							sleep( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS );
						} // end show 3D real-time



						// ----------------------------------------
						// RUN ONE STEP OF THE PARTICLE FILTER:
						// ----------------------------------------
						tictac.Tic();
						if (!SAVE_STATS_ONLY)
							printf("Step %u -- Executing ParticleFilter on %u particles....",(unsigned int)step, (unsigned int)pdf.particlesCount());

						PF.executeOn(
							pdf,
							action.pointer(),			// Action
							observations.pointer(),	// Obs.
							&PF_stats		// Output statistics
							);

						if (!SAVE_STATS_ONLY)
							printf(" Done! in %.03fms, ESS=%f\n", 1000.0f*tictac.Tac(), pdf.ESS());
					}

					// Avrg. error:
					// ----------------------------------------
					odometryEstimation = odometryEstimation + action->getBestMovementEstimation()->poseChange->getMeanVal();
					pdf.getMean( pdfEstimation );

					getGroundTruth(expectedPose, rawlogEntry, GT, cur_obs_timestamp );
					//cout << format("TIM: %f GT:", mrpt::system::timestampTotime_t(cur_obs_timestamp)) << expectedPose << endl;

#if 1
					{	// Averaged error to GT
						double sumW=0;
						double locErr=0;
						for (size_t k=0;k<pdf.size();k++) sumW+=exp(pdf.getW(k));
						for (size_t k=0;k<pdf.size();k++)
							locErr+= expectedPose.distanceTo( pdf.getParticlePose(k) ) * exp(pdf.getW(k))/ sumW;
						covergenceErrors.push_back( locErr );
					}
#else
					// Error of the mean to GT
					covergenceErrors.push_back( expectedPose.distanceTo( pdfEstimation ) );
#endif

					// Text output:
					// ----------------------------------------
					if (!SAVE_STATS_ONLY)
					{
						cout << "    Odometry est: " << odometryEstimation << "\n";
						cout << "         PDF est: " << pdfEstimation << ", ESS (B.R.)= " << PF_stats.ESS_beforeResample << "\n";
						if (GT.getRowCount()>0)
							cout << "    Ground truth: " << expectedPose << "\n";
					}

					pdf.getCovariance(cov);

					if (!SAVE_STATS_ONLY)
					{
			                        f_cov_est.printf("%e\n",sqrt(cov.det()) );
			                        f_pf_stats.printf("%u %e %e\n",
							(unsigned int)pdf.size(),
							PF_stats.ESS_beforeResample,
							PF_stats.weightsVariance_beforeResample );
			                        f_odo_est.printf("%f %f %f\n",odometryEstimation.x(),odometryEstimation.y(),odometryEstimation.phi());
					}

					CPose2D meanPose;
					CMatrixDouble33 cov;
					pdf.getCovarianceAndMean(cov,meanPose);

					if ( !SAVE_STATS_ONLY && SCENE3D_FREQ>0 && (step % SCENE3D_FREQ)==0)
					{
						// Generate 3D scene:
						// ------------------------------

						// The Ground Truth (GT):
						{
							CRenderizablePtr GTpt = scene.getByName("GT");
							if (!GTpt)
							{
								GTpt = CDisk::Create();
								GTpt = CDisk::Create();
								GTpt->setName( "GT" );
								GTpt->setColor(0,0,0, 0.9);

								getAs<CDisk>(GTpt)->setDiskRadius(0.04);
								scene.insert( GTpt );
							}

							GTpt->setPose(expectedPose);
						}

						// The particles:
						{
							CRenderizablePtr parts = scene.getByName("particles");
							if (parts) scene.removeObject(parts);

							CSetOfObjectsPtr p = pdf.getAs3DObject<CSetOfObjectsPtr>();
							p->setName("particles");
							scene.insert(p);
						}

						// The particles' cov:
						{
							CRenderizablePtr	ellip = scene.getByName("parts_cov");
							if (!ellip)
							{
								ellip = CEllipsoid::Create();
								ellip->setName( "parts_cov");
								ellip->setColor(1,0,0, 0.6);

								getAs<CEllipsoid>(ellip)->setLineWidth(2);
								getAs<CEllipsoid>(ellip)->setQuantiles(3);
								scene.insert( ellip );
							}
							ellip->setLocation(meanPose.x(),meanPose.y(),0);

							getAs<CEllipsoid>(ellip)->setCovMatrix(cov,2);
						}


						// The laser scan:
						{
							CRenderizablePtr scanPts = scene.getByName("scan");
							if (!scanPts)
							{
								scanPts = CPointCloud::Create();
								scanPts->setName( "scan" );
								scanPts->setColor(1,0,0, 0.9);
								getAs<CPointCloud>(scanPts)->enableColorFromZ(false);
								getAs<CPointCloud>(scanPts)->setPointSize(4);
								scene.insert(scanPts);
							}

							CSimplePointsMap	map;
							static CSimplePointsMap	last_map;

							CPose3D				robotPose3D( meanPose );

							map.clear();
							observations->insertObservationsInto( &map );

							getAs<CPointCloud>(scanPts)->loadFromPointsMap( &last_map );
							getAs<CPointCloud>(scanPts)->setPose( robotPose3D );
							last_map = map;
						}

						// The camera:
						scene.enableFollowCamera(SCENE3D_FOLLOW);

						// Views:
						COpenGLViewportPtr view1= scene.getViewport("main");
						{
							CCamera  &cam = view1->getCamera();
							cam.setAzimuthDegrees(-90);
							cam.setElevationDegrees(90);
							cam.setPointingAt( meanPose);
							cam.setZoomDistance(5);
							cam.setOrthogonal();
						}

						COpenGLViewportPtr view2= scene.createViewport("small_view"); // Create, or get existing one.
						view2->setCloneView("main");
						view2->setCloneCamera(false);
						view2->setBorderSize(3);
						{
							CCamera  &cam = view1->getCamera();
							cam.setAzimuthDegrees(-90);
							cam.setElevationDegrees(90);
							cam.setPointingAt( meanPose );
							cam.setZoomDistance(15);
							cam.setOrthogonal();

							view2->setTransparent(false);
							view2->setViewportPosition(0.59,0.01,0.4,0.3);
						}
					}

					if (!SAVE_STATS_ONLY && SCENE3D_FREQ!=-1 && (step % SCENE3D_FREQ)==0)
					{
						// Save 3D scene:
						CFileGZOutputStream(format("%s/progress_%03u.3Dscene",OUT_DIR_3D,(unsigned)step)) << scene;

						// Generate text files for matlab:
						// ------------------------------------
						pdf.saveToTextFile(format("%s/particles_%03u.txt",OUT_DIR_PARTS,(unsigned)step));

						if (IS_CLASS(*observations->begin(),CObservation2DRangeScan))
						{
							CObservation2DRangeScanPtr o = CObservation2DRangeScanPtr( *observations->begin() );
							vectorToTextFile(o->scan , format("%s/observation_scan_%03u.txt",OUT_DIR_PARTS,(unsigned)step) );
						}
					}

				} // end if rawlog_offset

				step++;

				// Test for end condition if we are testing convergence:
				if ( step == testConvergenceAt )
				{
					nConvergenceTests++;

					// Convergence??
					if ( sqrt(cov.det()) < 2 )
					{
						if ( pdfEstimation.distanceTo(expectedPose) < 1.00f )
							nConvergenceOK++;
					}
					end = true;
				}
			}; // while rawlogEntries

		} // for repetitions

		double repetitionTime = tictacGlobal.Tac();

		// Avr. error:
		double covergenceErrorMean, covergenceErrorsMin,covergenceErrorsMax;
		math::condidenceIntervals(covergenceErrors, covergenceErrorMean, covergenceErrorsMin,covergenceErrorsMax, STATS_CONF_INTERVAL);

		// Save overall results:
		{

			CFileOutputStream f(format("%s_SUMMARY.txt",OUT_DIR_PREFIX.c_str()), true /* append */);

			f.printf("%% Ratio_covergence_success  #particles  average_time_per_execution  convergence_mean_error convergence_error_conf_int_inf convergence_error_conf_int_sup \n");
			if (!nConvergenceTests) nConvergenceTests=1;
			f.printf("%f %u %f %f %f %f\n",
				((double)nConvergenceOK)/nConvergenceTests,
				PARTICLE_COUNT,
				repetitionTime /NUM_REPS,
				covergenceErrorMean,
				covergenceErrorsMin,covergenceErrorsMax );
		}

		printf("\n TOTAL EXECUTION TIME = %.06f sec\n", repetitionTime );

	} // end of loop for different # of particles

	if (win3D)
		mrpt::system::pause();
}


void getGroundTruth( CPose2D &expectedPose, size_t rawlogEntry, const CMatrixDouble &GT, const TTimeStamp &cur_time)
{
	if (GT.getColCount()==4)
	{
		static bool first_step = true;
		static bool GT_index_is_time;

		// First column can be: timestamps, or rawlogentries:
		//  Auto-figure it out:
		if (GT.getRowCount()>2)
		{
			GT_index_is_time = floor(GT(0,0))!=GT(0,0) && floor(GT(1,0))!=GT(1,0);
		}
		else
		{
			GT_index_is_time = false;
		}

		if (GT_index_is_time)
		{
			// Look for the timestamp:
			static std::map<double,CPose2D>	GT_path;
			std::map<double,CPose2D>::iterator it;
			if (first_step)
			{
				for (size_t i=0;i<GT.getRowCount();i++)
					GT_path[ mrpt::math::round_10power(GT(i,0),-4) ] = CPose2D(GT(i,1),GT(i,2),GT(i,3));
			}

			double TT =mrpt::system::timestampTotime_t(cur_time);
			double T = mrpt::math::round_10power( TT, -4);

			it = GT_path.find(T);
			if (it!=GT_path.end())
			{
				expectedPose = it->second;
			}
			else cout << format("GT time not found: %f\n", T);
		}
		else
		{
			// Look for the rawlogEntry:
			size_t  k, N = GT.getRowCount();
			for (k=0;k<N;k++)
			{
				if (GT(k,0)==rawlogEntry )
					break;
			}

			if (k<N)
			{
				expectedPose.x(GT(k,1));
				expectedPose.y(GT(k,2));
				expectedPose.phi(GT(k,3));
			}
		}
		first_step=false;
	}
	else
	if (GT.getColCount()==3)
	{
		if ( rawlogEntry<GT.getRowCount() )
		{
			expectedPose.x(GT(rawlogEntry,0) );
			expectedPose.y( GT(rawlogEntry,1) );
			expectedPose.phi( GT(rawlogEntry,2) );
		}
	}
	else if (GT.getColCount()>0) THROW_EXCEPTION("Unexpected number of columns in ground truth file");
}

