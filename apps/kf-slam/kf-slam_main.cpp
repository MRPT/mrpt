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
    APPLICATION: Kalman Filter-based SLAM implementation
    FILE: kf-slam_main.cpp
    AUTHOR: Jose Luis Blanco Claraco <jlblanco@ctima.uma.es>

	See README.txt for instructions.
 ---------------------------------------------------------------*/

#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

std::string		configFile;

void Run_KF_SLAM();

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" KF-SLAM version 0.2 - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2)
		{
			printf("Use: kf-slam <config_file>\n\nPush any key to exit...\n");
			os::getch();
			return -1;
		}

		configFile = std::string( argv[1] );

		Run_KF_SLAM();

		return 0;
	}
	catch (std::exception &e)
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
//				TestMapping
// ------------------------------------------------------
void Run_KF_SLAM()
{
	CRangeBearingKFSLAM  mapping;
	CConfigFile			 cfgFile( configFile );
	std::string			 rawlogFileName;

	//mapping.debugOut = &myDebugStream;

	// The rawlog file:
	// ----------------------------------------
	rawlogFileName = cfgFile.read_string("MappingApplication","rawlog_file",std::string("log.rawlog"));
	unsigned int	rawlog_offset = cfgFile.read_int("MappingApplication","rawlog_offset",0);

	unsigned int SAVE_LOG_FREQUENCY= cfgFile.read_int("MappingApplication","SAVE_LOG_FREQUENCY",1);

	bool  SAVE_3D_SCENES = cfgFile.read_bool("MappingApplication","SAVE_3D_SCENES", true);
	bool  SAVE_MAP_REPRESENTATIONS = cfgFile.read_bool("MappingApplication","SAVE_MAP_REPRESENTATIONS", true);
	bool  SHOW_3D_LIVE = cfgFile.read_bool("MappingApplication","SHOW_3D_LIVE", false);
	
	bool  FORCE_IGNORE_ODOMETRY = cfgFile.read_bool("MappingApplication","FORCE_IGNORE_ODOMETRY", false);


	string OUT_DIR = cfgFile.read_string("MappingApplication","logOutput_dir","OUT_KF-SLAM");
	string ground_truth_file = cfgFile.read_string("MappingApplication","ground_truth_file","");
	string ground_truth_file_robot= cfgFile.read_string("MappingApplication","ground_truth_file_robot","");

	cout << "RAWLOG FILE:" << endl << rawlogFileName << endl;
	ASSERT_( fileExists( rawlogFileName ) );
	CFileGZInputStream	rawlogFile( rawlogFileName );

	cout << "---------------------------------------------------" << endl << endl;

	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	// Load the config options for mapping:
	// ----------------------------------------
	mapping.loadOptions( CConfigFile(configFile) );
	mapping.KF_options.dumpToConsole();
	mapping.options.dumpToConsole();

	// debug:
	//mapping.KF_options.use_analytic_observation_jacobian = true;
	//mapping.KF_options.use_analytic_transition_jacobian = true;
	//mapping.KF_options.debug_verify_analytic_jacobians = true;


    // Is there ground truth of the robot poses??
	CMatrixDouble GT_PATH(0,0);
	if (ground_truth_file_robot.size() && fileExists(ground_truth_file_robot))
	{
		GT_PATH.loadFromTextFile(ground_truth_file_robot);
		ASSERT_(size(GT_PATH,1)>0 && size(GT_PATH,2)==6)
	}

	// Init 3D window:
	mrpt::gui::CDisplayWindow3DPtr win3d;

	if (SHOW_3D_LIVE)
	{
		win3d = mrpt::gui::CDisplayWindow3D::Create("KF-SLAM live view",800,500);
		
		win3d->addTextMessage(0.01,0.96,"Red: Estimated path",TColorf(0.8,0.8,0.8),100,MRPT_GLUT_BITMAP_HELVETICA_10);
		win3d->addTextMessage(0.01,0.93,"Black: Ground truth path",TColorf(0.8,0.8,0.8),101,MRPT_GLUT_BITMAP_HELVETICA_10);
	}

	//			INITIALIZATION
	// ----------------------------------------
	//mapping.initializeEmptyMap();

	// The main loop:
	// ---------------------------------------
	CActionCollectionPtr	action;
	CSensoryFramePtr		observations;
	size_t			rawlogEntry = 0, step = 0;

	typedef vector<CPose3DQuat, Eigen::aligned_allocator<CPose3DQuat> > TListCPose3DQuat;
	TListCPose3DQuat  meanPath; // The estimated path

	CPose3DQuatPDFGaussian		robotPose;
	std::vector<CPoint3D>	LMs;
	std::map<unsigned int,CLandmark::TLandmarkID> LM_IDs;
	CMatrixDouble	fullCov;
	CVectorDouble           fullState;

	for (;;)
	{
		if (os::kbhit())
        {
			char	pushKey = os::getch();
			if (27 == pushKey)
				break;
        }


		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::readActionObservationPair( rawlogFile, action, observations, rawlogEntry) )
			break; // file EOF

		if (rawlogEntry>=rawlog_offset)
		{
			// Process the action and observations:
			// --------------------------------------------
			static CTicTac kftictac;
			kftictac.Tic();

			mapping.processActionObservation(action,observations);

			const double tim_kf_iter = kftictac.Tac();

			// Get current state:
			// -------------------------------
			mapping.getCurrentState( robotPose,LMs,LM_IDs,fullState,fullCov );
			cout << "Mean pose: " << endl << robotPose.mean << endl;
			cout << "# of landmarks in the map: " << LMs.size() << endl;

			// Build the path:
			meanPath.push_back( robotPose.mean );

			// Save mean pose:
			if (!(step % SAVE_LOG_FREQUENCY))
			{
				const CMatrixDouble71  p= CMatrixDouble71(robotPose.mean);
				p.saveToTextFile(OUT_DIR+format("/robot_pose_%05u.txt",(unsigned int)step));
			}

			// Save full cov:
			if (!(step % SAVE_LOG_FREQUENCY))
			{
				fullCov.saveToTextFile(OUT_DIR+format("/full_cov_%05u.txt",(unsigned int)step));
			}

			// Save map to file representations?
			if (SAVE_MAP_REPRESENTATIONS  && !(step % SAVE_LOG_FREQUENCY))
			{
				mapping.saveMapAndPath2DRepresentationAsMATLABFile( OUT_DIR+format("/slam_state_%05u.m",(unsigned int)step) );
			}

			// Save 3D view of the filter state:
			if (win3d.present() || ( SAVE_3D_SCENES && !(step % SAVE_LOG_FREQUENCY) ) )
			{
				COpenGLScenePtr   scene3D = COpenGLScene::Create();
				{
					opengl::CGridPlaneXYPtr grid = opengl::CGridPlaneXY::Create(-1000,1000,-1000,1000,0,5);
					grid->setColor(0.4,0.4,0.4);
					scene3D->insert( grid );
				}

				// Robot path:
				{
					opengl::CSetOfLinesPtr linesPath = opengl::CSetOfLines::Create();
					linesPath->setColor(1,0,0);

					double x0=0,y0=0,z0=0;
					if (!meanPath.empty())
					{
						x0 = meanPath[0].x();
						y0 = meanPath[0].y();
						z0 = meanPath[0].z();
					}

					int path_decim = 0;
					for (TListCPose3DQuat::iterator it=meanPath.begin();it!=meanPath.end();++it)
					{
						linesPath->appendLine(
							x0,y0,z0,
							it->x(), it->y(), it->z() );
						x0=it->x();
						y0=it->y();
						z0=it->z();

						if (++path_decim>10)
						{
							path_decim = 0;
							mrpt::opengl::CSetOfObjectsPtr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(0.3f,2.0f);
							xyz->setPose(CPose3D(*it));
							scene3D->insert(xyz);
						}
					}
					scene3D->insert( linesPath );
					
					// finally a big corner for the latest robot pose:
					{
						mrpt::opengl::CSetOfObjectsPtr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(1.0,2.5);
						xyz->setPose(CPose3D(robotPose.mean));
						scene3D->insert(xyz);
					}

				}

				{
					// Do we have a ground truth?
					if (size(GT_PATH,2)==6)
					{
						opengl::CSetOfLinesPtr GT_path = opengl::CSetOfLines::Create();
						GT_path->setColor(0,0,0);

						double gtx0=0,gty0=0,gtz0=0;
						size_t N = std::min(size(GT_PATH,1), meanPath.size() );
						for (size_t i=0;i<N;i++)
						{
							const CPose3D  p( GT_PATH(i,0),GT_PATH(i,1),GT_PATH(i,2), GT_PATH(i,3),GT_PATH(i,4),GT_PATH(i,5) );

							GT_path->appendLine(
								gtx0,gty0,gtz0,
								p.x(),p.y(),p.z() );
							gtx0=p.x();
							gty0=p.y();
							gtz0=p.z();
						}
						scene3D->insert( GT_path );
					}
				}

				// Draw latest data association:
				{
					const CRangeBearingKFSLAM::TDataAssocInfo & da = mapping.getLastDataAssociation();

					mrpt::opengl::CSetOfLinesPtr lins = mrpt::opengl::CSetOfLines::Create();
					lins->setLineWidth(1.2);
					lins->setColor(1,1,1);
					for (std::map<observation_index_t,prediction_index_t>::const_iterator it=da.results.associations.begin();it!=da.results.associations.end();++it)
					{
						const prediction_index_t idxPred = it->second;
						// This index must match the internal list of features in the map:
						CRangeBearingKFSLAM::KFArray_FEAT featMean;
						mapping.getLandmarkMean(idxPred, featMean);

						// Line: robot -> landmark:
						lins->appendLine(
							robotPose.mean.x(),robotPose.mean.y(),robotPose.mean.z(),
							featMean[0],featMean[1],featMean[2]);
					}
					scene3D->insert( lins );
				}

				// The current state of KF-SLAM:
				{
					opengl::CSetOfObjectsPtr  objs = opengl::CSetOfObjects::Create();
					mapping.getAs3DObject(objs);
					scene3D->insert( objs );
				}

				if (win3d.present())
				{
					mrpt::opengl::COpenGLScenePtr &scn = win3d->get3DSceneAndLock();
					scn = scene3D;

					// Update text messages:
					win3d->addTextMessage(
						0.02,0.02, 
						format("Step %u",(unsigned int)step ), 
						TColorf(1,1,1), 0, MRPT_GLUT_BITMAP_HELVETICA_12 );

					win3d->addTextMessage(
						0.02,0.06, 
						format("Estimated pose: (x y z qr qx qy qz) = %s", robotPose.mean.asString().c_str() ), 
						TColorf(1,1,1), 1, MRPT_GLUT_BITMAP_HELVETICA_12 );

					static vector<double> estHz_vals;
					const double curHz = 1.0/std::max(1e-9,tim_kf_iter);
					estHz_vals.push_back(curHz);
					if (estHz_vals.size()>50)
						estHz_vals.erase(estHz_vals.begin());
					const double meanHz = mrpt::math::mean(estHz_vals);

									
					win3d->addTextMessage(
						0.02,0.10, 
						format("Iteration time: %7ss", 
							mrpt::utils::unitsFormat(tim_kf_iter).c_str()),
							TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );

					win3d->addTextMessage(
						0.02,0.14, 
						format("Execution rate: %7sHz", 
							mrpt::utils::unitsFormat(meanHz).c_str()),
							TColorf(1,1,1), 3, MRPT_GLUT_BITMAP_HELVETICA_12 );

					win3d->unlockAccess3DScene();
					win3d->repaint();
				}

				if ( SAVE_3D_SCENES && !(step % SAVE_LOG_FREQUENCY) )
				{
					// Save to file:
					CFileGZOutputStream(OUT_DIR+format("/kf_state_%05u.3Dscene",(unsigned int)step)) << *scene3D;
				}
			}


			// Free rawlog items memory:
			// --------------------------------------------
			action.clear_unique();
			observations.clear_unique();

		} // (rawlogEntry>=rawlog_offset)

		cout << format("\nStep %u  - Rawlog entries processed: %i\n", (unsigned int)step, (unsigned int)rawlogEntry);

        step++;
	};	// end "while(1)"


    // Compute the "information" between partitions:
    if (mapping.options.doPartitioningExperiment)
    {
		// --------------------------------------------
		// PART I:
		//  Comparison to fixed partitioning every K obs.
		// --------------------------------------------

        // Compute the information matrix:
        size_t i;
        for (i=0;i<6;i++) fullCov(i,i) = max(fullCov(i,i), 1e-6);

        CMatrix		H( fullCov.inv() );
        H.saveToTextFile(OUT_DIR+string("/information_matrix_final.txt"));

        // Replace by absolute values:
        H.Abs();
        CMatrix H2(H); H2.normalize(0,1);
        CImageFloat   imgF(H2);
        imgF.saveToFile(OUT_DIR+string("/information_matrix_final.png"));


        // ----------------------------------------
        // Compute the "approximation error factor" E:
        //  E = SUM() / SUM(ALL ELEMENTS IN MATRIX)
        // ----------------------------------------
        vector<vector_uint>  landmarksMembership,partsInObsSpace;
        CMatrix  ERRS(50,3);

        for (i=0;i<ERRS.getRowCount();i++)
        {
            size_t K;

            if (i==0)
            {
                K=0;
                mapping.getLastPartitionLandmarks( landmarksMembership );
            }
            else
            {
                K=i+1;
                mapping.getLastPartitionLandmarksAsIfFixedSubmaps(i+1,landmarksMembership);
            }

            mapping.getLastPartition(partsInObsSpace);

            ERRS(i,0) = (float)K;
            ERRS(i,1) = (float)partsInObsSpace.size();
            ERRS(i,2) = mapping.computeOffDiagonalBlocksApproximationError(landmarksMembership);
        }


        ERRS.saveToTextFile( OUT_DIR+string("/ERRORS.txt" ));
        //printf("Approximation error from partition:\n"); cout << ERRS << endl;

		// --------------------------------------------
		// PART II:
		//  Sweep partitioning threshold:
		// --------------------------------------------
		size_t STEPS = 50;
        CVectorFloat	ERRS_SWEEP(STEPS),ERRS_SWEEP_THRESHOLD(STEPS);

		// Compute the error for each partitioning-threshold
        for (i=0;i<STEPS;i++)
        {
			float th = (1.0f*i)/(STEPS-1.0f);
			ERRS_SWEEP_THRESHOLD[i] = th;
			mapping.mapPartitionOptions()->partitionThreshold =  th;

			mapping.reconsiderPartitionsNow();

			mapping.getLastPartitionLandmarks( landmarksMembership );
            ERRS_SWEEP[i] = mapping.computeOffDiagonalBlocksApproximationError(landmarksMembership);
        }

        ERRS_SWEEP.saveToTextFile( OUT_DIR+string("/ERRORS_SWEEP.txt" ));
        ERRS_SWEEP_THRESHOLD.saveToTextFile( OUT_DIR+string("/ERRORS_SWEEP_THRESHOLD.txt" ));

    } // end if doPartitioningExperiment


    // Is there ground truth of landmarks positions??
    if (ground_truth_file.size() && fileExists(ground_truth_file))
    {
        CMatrixFloat    GT(0,0);
        try
        {
            GT.loadFromTextFile(ground_truth_file);
        }
        catch(std::exception &e)
        {
            cerr << "Ignoring the following error loading ground truth file: " << e.what() << endl;
        }

        if (GT.getRowCount()>0 && LMs.size())
        {
            // Each row has:
            //   [0] [1] [2]  [6]
            //    x   y   z    ID
            vector_double ERRS(0);
            for (size_t i=0;i<LMs.size();i++)
            {
                // Find the entry in the GT for this mapped LM:
                bool found = false;
                for (size_t r=0;r<GT.getRowCount();r++)
                {
                    if ( LM_IDs[i] == GT(r,6) )
                    {
                        ERRS.push_back( LMs[i].distance3DTo( GT(r,0),GT(r,1),GT(r,2) ) );
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    cerr << "Ground truth entry not found for landmark ID:" << LM_IDs[i] << endl;
                }
            }

            printf("ERRORS VS. GROUND TRUTH:\n");
            printf("Mean Error: %f meters\n", math::mean(ERRS) );
            printf("Minimum error: %f meters\n", math::minimum(ERRS) );
            printf("Maximum error: %f meters\n", math::maximum(ERRS) );
        }
    } // end if GT

	cout << "********* KF-SLAM finished! **********" << endl;

	if (win3d) 
	{
		cout << "\n Close the 3D window to quit the application.\n";
		win3d->waitForKey();
	}

}

