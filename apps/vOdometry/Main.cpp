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
#include <mrpt/slam.h>
#include <mrpt/stereoslam.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace mrpt::stereoslam;
using namespace std;

//*****************************************************
//			Config params
//*****************************************************/
string					INI_FILENAME;
utils::CConfigFile		*iniFile = NULL;

string					RAWLOG_FILE;
unsigned int			RAWLOG_OFFSET;
string					OUT_DIR_STD;
const char				*OUT_DIR;
int						LOG_FREQUENCY;
unsigned int			DECIMATION;

// ------------------------------------------------------
//					Visual Odometry
// ------------------------------------------------------
void vOdometry_lightweight()
{
	// My Local Variables
	CVisualOdometryStereo				vOdometer;
	unsigned int						step = 0;
	CTicTac								tictac;

	std::vector<CPose3DQuat>			path1;
	std::vector<CPose3DQuatPDFGaussian>	path2;

	size_t								rawlogEntry = 0;
	CFileGZInputStream					rawlogFile( RAWLOG_FILE );

	// Initial pose of the path
	path1.push_back( CPose3DQuat() );
	path2.push_back( CPose3DQuatPDFGaussian() );

	// ----------------------------------------------------------
	//						vOdometry
	// ----------------------------------------------------------
	CActionCollectionPtr	action;
	CSensoryFramePtr		observations;
	CObservationPtr			observation;

	// Load options (stereo + matching + odometry)
	vOdometer.loadOptions( INI_FILENAME );

	// Delete previous files and prepare output dir
	deleteFiles( format("%s/*.txt", OUT_DIR) );

	FILE *f_cov = os::fopen( format( "%s/cov.txt", OUT_DIR ), "wt");
	ASSERT_( f_cov != NULL );

	// Iteration counter
	int	counter = 0;

	FILE *f_log = os::fopen( format( "%s/q.txt", OUT_DIR ), "wt");
	FILE *f_log2 = os::fopen( format( "%s/path.txt", OUT_DIR ), "wt");

	unsigned int imDecimation = 5;

	// Main Loop
	tictac.Tic();
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
		if (! CRawlog::getActionObservationPairOrObservation( rawlogFile, action, observations, observation, rawlogEntry) )
			break; // file EOF


		if ( rawlogEntry >= RAWLOG_OFFSET && 0 == ( step % DECIMATION ) )
		{
			// Execute:
			// ----------------------------------------

			// STEREO IMAGES OBSERVATION
			CObservationStereoImagesPtr sImgs;
			if( observation )
				sImgs = CObservationStereoImagesPtr( observation );
			else 
				sImgs = observations->getObservationByClass<CObservationStereoImages>();
			
			poses::CPose3DQuatPDFGaussian outEst;
			if( sImgs )
			{
				if( !counter )
				{
					// Set initial parameters
					vOdometer.stereoParams.baseline = sImgs->rightCameraPose.x();
					vOdometer.stereoParams.K = sImgs->leftCamera.intrinsicParams;
				}

				cout << "Rawlog Entry: " << rawlogEntry << " Iteration: " << counter++ << endl;

				if( step % imDecimation )
				{
					vOdometer.process_light( sImgs, outEst );
					TOdometryInfo info = vOdometer.getInfo();

					// Save to file both the quaternion and covariance matrix
					os::fprintf( f_log,"%f %f %f %f %f %f %f\n", 
						outEst.mean[0], outEst.mean[1], outEst.mean[2], outEst.mean[3], outEst.mean[4], outEst.mean[5], outEst.mean[6] );
					info.m_Prev_cloud.landmarks.saveToTextFile( format( "%s/clouds%04d.txt", OUT_DIR, step ) );

					path1.push_back( path1.back() + outEst.mean );
					os::fprintf( f_log2,"%f %f %f %f %f %f %f\n", 
						path1.back()[0], path1.back()[1], path1.back()[2], path1.back()[3], path1.back()[4], path1.back()[5], path1.back()[6] );

				path2.push_back( outEst );
				}
				else
					cout << "Skipped step" << endl;

			} // end if sImgs != NULL

		} // end if 'rawlogEntry >= rawlog_offset'

		step++;

		// Free memory:
		action.clear_unique();
		observations.clear_unique();

	}; // end  while !end
	cout << "*************** Tiempo: " << 1000.0f*tictac.Tac() << "************************" << endl;

	os::fclose( f_cov );
	os::fclose( f_log );
	os::fclose( f_log2 );

	// SAVE THE RESULTS
	/**/
	FILE *fPath1 = os::fopen( format("./%s/EstimatedPath.txt", OUT_DIR).c_str(), "wt");
	if( fPath1 != NULL )
	{
		std::vector<CPose3DQuat>::iterator	itPath;
		for(itPath = path1.begin(); itPath != path1.end(); ++itPath )
			os::fprintf( fPath1,"%f %f %f %f %f %f %f\n", 
			itPath->x(), itPath->y(), itPath->z(), 
			itPath->quat().r(), itPath->quat().x(), itPath->quat().y(), itPath->quat().z() );

		os::fclose( fPath1 );
	}
	else
		std::cout << "WARNING: The estimated path could not be saved" << std::endl;

	FILE *fPath2 = os::fopen( format("./%s/EstimatedPathPDF.txt", OUT_DIR).c_str(), "wt");
	if( fPath2 != NULL )
	{
		std::vector<CPose3DQuatPDFGaussian>::iterator	itPath;
		for(itPath = path2.begin(); itPath != path2.end(); ++itPath )
		{
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", 
				itPath->mean.x(), itPath->mean.y(), itPath->mean.z(), 
				itPath->mean.quat().r(), itPath->mean.quat().x(), itPath->mean.quat().y(), itPath->mean.quat().z() );

			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(0,0), itPath->cov.get_unsafe(0,1), itPath->cov.get_unsafe(0,2), itPath->cov.get_unsafe(0,3), itPath->cov.get_unsafe(0,4), itPath->cov.get_unsafe(0,5), itPath->cov.get_unsafe(0,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(1,0), itPath->cov.get_unsafe(1,1), itPath->cov.get_unsafe(1,2), itPath->cov.get_unsafe(1,3), itPath->cov.get_unsafe(1,4), itPath->cov.get_unsafe(1,5), itPath->cov.get_unsafe(1,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(2,0), itPath->cov.get_unsafe(2,1), itPath->cov.get_unsafe(2,2), itPath->cov.get_unsafe(2,3), itPath->cov.get_unsafe(2,4), itPath->cov.get_unsafe(2,5), itPath->cov.get_unsafe(2,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(3,0), itPath->cov.get_unsafe(3,1), itPath->cov.get_unsafe(3,2), itPath->cov.get_unsafe(3,3), itPath->cov.get_unsafe(3,4), itPath->cov.get_unsafe(3,5), itPath->cov.get_unsafe(3,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(4,0), itPath->cov.get_unsafe(4,1), itPath->cov.get_unsafe(4,2), itPath->cov.get_unsafe(4,3), itPath->cov.get_unsafe(4,4), itPath->cov.get_unsafe(4,5), itPath->cov.get_unsafe(4,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(5,0), itPath->cov.get_unsafe(5,1), itPath->cov.get_unsafe(5,2), itPath->cov.get_unsafe(5,3), itPath->cov.get_unsafe(5,4), itPath->cov.get_unsafe(5,5), itPath->cov.get_unsafe(5,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(6,0), itPath->cov.get_unsafe(6,1), itPath->cov.get_unsafe(6,2), itPath->cov.get_unsafe(6,3), itPath->cov.get_unsafe(6,4), itPath->cov.get_unsafe(6,5), itPath->cov.get_unsafe(6,6));
		}
		os::fclose( fPath2 );
	}
	else
		std::cout << "WARNING: The estimated pdf path could not be saved" << std::endl;

	std::cout << "Saved results!" << std::endl;
	/**/

	mrpt::system::pause();

}

// ------------------------------------------------------
//					Visual Odometry
// ------------------------------------------------------
//void vOdometry()
//{
//	// My Local Variables
//	CVisualOdometryStereo				vOdometer;
//	unsigned int						step = 0;
//
//	std::vector<CPose3DQuat>			path;
//
//	// Other variables
//	CTicTac								tictac;
//
//	size_t								rawlogEntry = 0;
//	CFileGZInputStream					rawlogFile( RAWLOG_FILE );
//
//	CDisplayWindow3D					win( "Visual Odometry", 640, 480 );
//	win.setPos( 700, 20 );
//
//	COpenGLScenePtr						&theScene = win.get3DSceneAndLock();
//
//	// OPENGL VARIABLES
//	{
//		opengl::CPointCloudPtr	obj = opengl::CPointCloud::Create();
//		obj->setName("path");
//		theScene->insert( obj );
//	}
//	{
//		opengl::CSetOfLinesPtr	obj = opengl::CSetOfLines::Create();
//		obj->setName("path_lines");
//		theScene->insert( obj );
//	}
//
//	{
//		opengl::CGridPlaneXYPtr	obj = opengl::CGridPlaneXY::Create(-0.25,0.25,-0.5,0.5,0,0.05);
//		obj->setColor(0.4,0.4,0.4);
//		theScene->insert( obj );
//	}
//	{
//		opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();
//		obj->setName("ellipse");
//		theScene->insert( obj );
//	}
//	{
//		opengl::CAxisPtr obj = opengl::CAxis::Create();
//		obj->setFrequency(5);
//		obj->enableTickMarks();
//		obj->setAxisLimits(-10,-10,-10, 10,10,10);
//		theScene->insert( obj );
//	}
//
//	win.setCameraElevationDeg( 25.0f );
//	win.setCameraAzimuthDeg( 25.0f );
//	win.setCameraZoom( 1.0f );
//
//	win.unlockAccess3DScene();
//
//	// Initial pose of the path
//	path.push_back( CPose3DQuat() );
//
//	// ----------------------------------------------------------
//	//						vOdometry
//	// ----------------------------------------------------------
//	CActionCollectionPtr	action;
//	CSensoryFramePtr		observations;
//
//	// Load options
//	vOdometer.stereoParams.loadFromConfigFile(*iniFile,"StereoParams");
//	vOdometer.stereoParams.dumpToConsole();
//	vOdometer.matchingOptions.loadFromConfigFile(*iniFile,"MatchingOptions"	);
//	vOdometer.matchingOptions.dumpToConsole();
//	vOdometer.odometryOptions.loadFromConfigFile(*iniFile,"OdometryOptions");
//	vOdometer.odometryOptions.dumpToConsole();
//
//	// Delete previous files
//	deleteFiles( format("%s/*.txt", OUT_DIR) );
//
//	FILE *f_cov = os::fopen( format( "%s/cov.txt", OUT_DIR ), "wt");
//	ASSERT_( f_cov != NULL );
//
//	// Main Loop
//	for (;;)
//	{
//		if (os::kbhit())
//		{
//			char c = os::getch();
//			if (c==27)
//				break;
//		}
//
//		// Load action/observation pair from the rawlog:
//		// --------------------------------------------------
//		if (! CRawlog::readActionObservationPair( rawlogFile, action, observations, rawlogEntry) )
//			break; // file EOF
//
//		if ( rawlogEntry >= rawlog_offset )
//		{
//			// Execute:
//			// ----------------------------------------
//
//			// STEREO IMAGES OBSERVATION
//			CObservationStereoImagesPtr sImgs = observations->getObservationByClass<CObservationStereoImages>();
//			poses::CPose3DQuatPDFGaussian outEst;
//			if( sImgs )
//			{
//				// Set initial parameters
//				vOdometer.stereoParams.baseline = sImgs->rightCameraPose.x();
//				sImgs->leftCamera.getIntrinsicParamsMatrix( vOdometer.stereoParams.K );
//
//				// Perform an iteration
//				if(rawlogEntry > 200)
				//	mrpt::system::pause();
//
//				vOdometer.process( sImgs, outEst );
//
//				CPose3D	thispose = path.at( path.size() - 1 ) + outEst.mean;
//				path.push_back( thispose );
//
//				os::fprintf(f_cov, "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
//							outEst.cov.get_unsafe(0,0), outEst.cov.get_unsafe(0,1), outEst.cov.get_unsafe(0,2),
//							outEst.cov.get_unsafe(1,0), outEst.cov.get_unsafe(1,1), outEst.cov.get_unsafe(1,2),
//							outEst.cov.get_unsafe(2,0), outEst.cov.get_unsafe(2,1), outEst.cov.get_unsafe(2,2));
//
//				COpenGLScenePtr &theScene = win.get3DSceneAndLock();
//
//				{	// COVARIANCE MATRIX
//					CMatrixDouble33 loc_cov = CMatrixDouble33( outEst.cov );
//					//outEst.cov.extractMatrix(0,0,loc_cov);
//					opengl::CEllipsoidPtr obj1 = static_cast<opengl::CEllipsoidPtr>(theScene->getByName("ellipse"));
//					obj1->setCovMatrix(loc_cov);
//					obj1->setPose(thispose);
//					obj1->enableDrawSolid3D(false);
//				}
//
//				{	// POINT CLOUD
//					std::vector<CPose3D>::reverse_iterator	rit = path.rbegin();
//					size_t									tam = path.size();
//
//					opengl::CPointCloudPtr obj1 = static_cast<opengl::CPointCloudPtr>(theScene->getByName("path"));
//
//					obj1->getArrayX().resize( tam );
//					obj1->getArrayY().resize( tam );
//					obj1->getArrayZ().resize( tam );
//
//					obj1->getArrayX()[tam-1]	= rit->x();
//					obj1->getArrayY()[tam-1]	= rit->y();
//					obj1->getArrayZ()[tam-1]	= rit->z();
//
//					obj1->setColor(1,0,0);
//					obj1->setPointSize(3);
//				}
//
//				{	// SET OF LINES
//					size_t									tam = path.size();
//
//					if( tam > 1 )
//					{
//						opengl::CSetOfLinesPtr obj1 = static_cast<opengl::CSetOfLinesPtr>(theScene->getByName("path_lines"));
//
//						std::vector<CPose3D>::reverse_iterator	rit = path.rbegin();
//
//						float x1,y1,z1;
//						x1 = rit->x();
//						y1 = rit->y();
//						z1 = rit->z();
//
//						rit++;
//
//						obj1->appendLine( rit->x(), rit->y(), rit->z(), x1, y1, z1 );
//
//						obj1->setColor(0,0,1);
//					} // end if
//				}
//
//				// UNLOCK SCENE
//				win.unlockAccess3DScene();
//				win.forceRepaint();
//
//				//if( rawlogEntry > 180 )
//				//	mrpt::system::pause();
//
//			} // end if sImgs != NULL
//
//			// DEBUG: PLOT IMAGES AND FEATURES!
//			/** /
//			// DIBUJAR LAS FEATURES
//			if( PLOT_IMG )
//			{
//				char str[10];
//				tmpImg1 = sImgs->imageLeft;
//				if( PLOT_INFO )
//				{
//					for( itSIFT = SIFTmatchL.begin(); itSIFT != SIFTmatchL.end(); itSIFT++)
//					{
//						os::sprintf( str, 10, "%d", (int)itSIFT->ID );
//						tmpImg1.textOut( (int)itSIFT->x + 3, (int)itSIFT->y, str, 0xFF00FF );
//						tmpImg1.cross( (int)itSIFT->x, (int)itSIFT->y, 0xFF0000, '+');
//					}
//
//				} // end if
//
//				wind1.setPos( 0, 400 );
//				wind1.showImage( tmpImg1 );
//
//				tmpImg2 = sImgs->imageRight;
//				if( PLOT_INFO )
//				{
//					//for( itKLT = KLTList2.begin(); itKLT != KLTList2.end(); itKLT++)
//					//	tmpImg2.cross( (int)itKLT->x, (int)itKLT->y, 0x0000FF, '+');
//					for( itSIFT = SIFTmatchR.begin(); itSIFT != SIFTmatchR.end(); itSIFT++)
//					{
//						os::sprintf( str, 10, "%d", (int)itSIFT->ID );
//						tmpImg2.textOut( (int)itSIFT->x + 3, (int)itSIFT->y, str, 0xFF00FF );
//						tmpImg2.cross( (int)itSIFT->x, (int)itSIFT->y, 0xFF0000, '+');
//					}
//
//				} // end if
//
//				wind2.setPos( 650, 400 );
//				wind2.showImage( tmpImg2 );
//			} // end if PLOT_IMGS
//			/ * */
//		} // end if 'rawlogEntry >= rawlog_offset'
//
//		step++;
//
//		// Free memory:
//		action.clear_unique();
//		observations.clear_unique();
//
//	}; // end  while !end
//
//	os::fclose( f_cov );
//
//	// SAVE THE RESULTS
//	FILE *fPath = os::fopen( format("./%s/EstimatedPath.txt", OUT_DIR).c_str(), "wt");
//
//	//if( fPath != NULL )
//	//{
//	//	std::vector<CPose3DQuat>::iterator	itPath;
//	//	for(itPath = path.begin(); itPath != path.end(); itPath++ )
//	//		os::fprintf( fPath,"%f %f %f %f %f %f %f\n", 
//	//			itPath->x(), itPath->y(), itPath->z(), 
//	//			itPath->quat().r(), itPath->quat().x(), itPath->quat().y(), itPath->quat().z() );
//
//	//	os::fclose( fPath );
//	//}
//
//	mrpt::system::pause();
//} // end vOdometry

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{

		// Process arguments:
		if (argc<2)
		{
			printf("Use: MapBuilding_from_Rawlog <CONFIG_FILE.INI>\n\n");
			return -1;
		}

		INI_FILENAME = std::string( argv[1] );
		ASSERT_(fileExists(INI_FILENAME));

		iniFile = new utils::CConfigFile( INI_FILENAME );

		// ------------------------------------------
		//			Load config from file:
		// ------------------------------------------
		RAWLOG_FILE			= iniFile->read_string("OdometryApplication", "rawlogFile", "");
		RAWLOG_OFFSET		= iniFile->read_int("OdometryApplication", "rawlogOffset", 0);
		OUT_DIR_STD			= iniFile->read_string("OdometryApplication", "logOutputDir", "LOG_VODOMETRY");
		LOG_FREQUENCY		= iniFile->read_int("OdometryApplication", "logFrequency", 10);
		DECIMATION			= iniFile->read_int("OdometryApplication", "decimation", 1);

		// Set relative path for externally-stored images in rawlogs:
		string	rawlog_images_path = extractFileDirectory( RAWLOG_FILE );
		rawlog_images_path+=extractFileName(RAWLOG_FILE);
		rawlog_images_path+="_Images";
		CImage::IMAGES_PATH_BASE = rawlog_images_path;		// Set it.

		// For ease!
		OUT_DIR = OUT_DIR_STD.c_str();
		createDirectory(OUT_DIR);

		// Checks:
		ASSERT_(RAWLOG_FILE.size()>0);
		ASSERT_(fileExists(RAWLOG_FILE));

		// Print params:
		printf(" Running with the following parameters:\n");
		printf(" RAWLOG file:'%s'\n", RAWLOG_FILE.c_str());
		printf(" Output directory:\t\t\t'%s'\n",OUT_DIR);
		printf(" Log record freq:\t\t\t%u\n",LOG_FREQUENCY);

		// Call to the vOdometry process
		// vOdometry();
		vOdometry_lightweight();

		delete iniFile;

		mrpt::system::pause();
		return 0;
	} catch (std::exception &e)
	{
		printf("%s",e.what());
		printf("Program finished for an exception!!\n");;
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		printf("Not handled exception!!");
		mrpt::system::pause();
		return -1;
	}
}

