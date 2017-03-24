/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/random.h>
#include <mrpt/gui/CDisplayWindow3D.h>

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSphere.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::vision;
using namespace std;


int main(int argc, char ** argv)
{
    try
    {
		bool showHelp    = argc>1 && !os::_strcmp(argv[1],"--help");
		bool showVersion = argc>1 && !os::_strcmp(argv[1],"--version");

		printf(" simul-landmarks - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());

		if (showVersion)
			return 0;	// Program end

		// Process arguments:
		if (argc<2 || showHelp )
		{
			printf("Usage: %s <config_file.ini>\n\n",argv[0]);
			if (!showHelp)
			{
				mrpt::system::pause();
				return -1;
			}
			else	return 0;
		}

		string INI_FILENAME = std::string( argv[1] );
		ASSERT_FILE_EXISTS_(INI_FILENAME)

		CConfigFile		ini( INI_FILENAME );


		const int random_seed = ini.read_int("Params","random_seed",0);

		if (random_seed!=0)
				randomGenerator.randomize(random_seed);
		else	randomGenerator.randomize();

        // Set default values:
        unsigned int nLandmarks = 3;
        unsigned int nSteps = 100;
        std::string 	outFile("out.rawlog");
        std::string 	outDir("OUT");

        float min_x =-5;
        float max_x =5;
        float min_y =-5;
        float max_y =5;
        float min_z =0;
        float max_z =0;

        float odometryNoiseXY_std = 0.001f;
        float odometryNoisePhi_std_deg = 0.01f;
		float odometryNoisePitch_std_deg = 0.01f;
		float odometryNoiseRoll_std_deg = 0.01f;

        float minSensorDistance = 0;
        float maxSensorDistance = 10;
        float fieldOfView_deg= 180.0f;

        float sensorPose_x = 0;
        float sensorPose_y = 0;
        float sensorPose_z = 0;
        float sensorPose_yaw_deg = 0;
        float sensorPose_pitch_deg = 0;
        float sensorPose_roll_deg = 0;

        float stdRange = 0.01f;
        float stdYaw_deg = 0.1f;
        float stdPitch_deg = 0.1f;

        bool sensorDetectsIDs=true;

        bool  circularPath = true;
		bool  random6DPath = false;
        size_t squarePathLength=40;

        // Load params from INI:
        MRPT_LOAD_CONFIG_VAR(outFile,string,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(outDir,string,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(nSteps,int,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(circularPath,bool,	ini,"Params");
		MRPT_LOAD_CONFIG_VAR(random6DPath,bool,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(squarePathLength,int,ini,"Params");

        MRPT_LOAD_CONFIG_VAR(sensorDetectsIDs,bool, ini,"Params");

        MRPT_LOAD_CONFIG_VAR(odometryNoiseXY_std,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(odometryNoisePhi_std_deg,float, ini,"Params");
		MRPT_LOAD_CONFIG_VAR(odometryNoisePitch_std_deg,float, ini,"Params");
		MRPT_LOAD_CONFIG_VAR(odometryNoiseRoll_std_deg,float, ini,"Params");

        MRPT_LOAD_CONFIG_VAR(sensorPose_x,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(sensorPose_y,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(sensorPose_z,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(sensorPose_yaw_deg,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(sensorPose_pitch_deg,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(sensorPose_roll_deg,float,	ini,"Params");


        MRPT_LOAD_CONFIG_VAR(minSensorDistance,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(maxSensorDistance,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(fieldOfView_deg,float,	ini,"Params");

		bool show_in_3d = false;
		MRPT_LOAD_CONFIG_VAR(show_in_3d,bool,	ini,"Params");


        MRPT_LOAD_CONFIG_VAR(stdRange,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(stdYaw_deg,float,	ini,"Params");
        MRPT_LOAD_CONFIG_VAR(stdPitch_deg,float,	ini,"Params");


        float stdYaw = DEG2RAD(stdYaw_deg);
        float stdPitch = DEG2RAD(stdPitch_deg);

        float odometryNoisePhi_std = DEG2RAD(odometryNoisePhi_std_deg);
        float odometryNoisePitch_std = DEG2RAD(odometryNoisePitch_std_deg);
        float odometryNoiseRoll_std = DEG2RAD(odometryNoiseRoll_std_deg);

        float fieldOfView = DEG2RAD(fieldOfView_deg);

        CPose3D  sensorPoseOnRobot(
            sensorPose_x,
            sensorPose_y,
            sensorPose_z,
            DEG2RAD( sensorPose_yaw_deg ),
            DEG2RAD( sensorPose_pitch_deg ),
            DEG2RAD( sensorPose_roll_deg ) );

        // Create out dir:
        mrpt::system::createDirectory(outDir);

        // ---------------------------------------------
        // Create the point-beacons:
        // ---------------------------------------------
        printf("Creating landmark map...");
        mrpt::maps::CLandmarksMap    landmarkMap;
        int randomSetCount = 0;
        int uniqueIds = 1;

        // Process each of the "RANDOMSET_%i" found:
        do
        {
            string sectName = format("RANDOMSET_%i",++randomSetCount);

            nLandmarks = 0;
            MRPT_LOAD_CONFIG_VAR(nLandmarks,uint64_t,	ini,sectName);
            MRPT_LOAD_CONFIG_VAR(min_x,float,	ini,sectName);
            MRPT_LOAD_CONFIG_VAR(max_x,float,	ini,sectName);
            MRPT_LOAD_CONFIG_VAR(min_y,float,	ini,sectName);
            MRPT_LOAD_CONFIG_VAR(max_y,float,	ini,sectName);
            MRPT_LOAD_CONFIG_VAR(min_z,float,	ini,sectName);
            MRPT_LOAD_CONFIG_VAR(max_z,float,	ini,sectName);

            for (size_t i=0;i<nLandmarks;i++)
            {
                CLandmark   LM;
                CPointPDFGaussian   pt3D;

                // Random coordinates:
                pt3D.mean = CPoint3D(
					randomGenerator.drawUniform(min_x,max_x),
					randomGenerator.drawUniform(min_y,max_y),
					randomGenerator.drawUniform(min_z,max_z) );

                // Add:
				LM.createOneFeature();
				LM.features[0]->type = featBeacon;
                LM.ID = uniqueIds++;
                LM.setPose( pt3D );

                landmarkMap.landmarks.push_back(LM);
            }

            if (nLandmarks) cout << nLandmarks << " generated for the 'randomset' " << randomSetCount << endl;

        }
        while (nLandmarks);


        landmarkMap.saveToTextFile( format("%s/%s_ground_truth.txt",outDir.c_str(),outFile.c_str()) );
        printf("Done!\n");

        // ---------------------------------------------
        // Simulate:
        // ---------------------------------------------
		size_t nWarningsNoSight=0; 

        CActionRobotMovement2D::TMotionModelOptions   opts;
        opts.modelSelection = CActionRobotMovement2D::mmGaussian;
        opts.gaussianModel.a1=0;
        opts.gaussianModel.a2=0;
        opts.gaussianModel.a3=0;
        opts.gaussianModel.a4=0;
        opts.gaussianModel.minStdXY = odometryNoiseXY_std;
        opts.gaussianModel.minStdPHI = odometryNoisePhi_std;

		// Output rawlog, gz-compressed.
		CFileGZOutputStream  fil( format("%s/%s",outDir.c_str(),outFile.c_str()));
        CPose3D         realPose;
        
		const size_t N_STEPS_STOP_AT_THE_BEGINNING = 4;

		CMatrixDouble  GT_path;

        for (size_t i=0;i<nSteps;i++)
        {
            cout << "Generating step " << i << "...\n";
            CSensoryFrame           SF;
            CActionCollection         acts;

	        CPose3D	incPose3D;
			bool    incPose_is_3D = random6DPath;

            if (i>=N_STEPS_STOP_AT_THE_BEGINNING)
            {
				if (random6DPath)
				{	// 3D path
					const double Ar = DEG2RAD(3);
					TPose3D  Ap = TPose3D(0.20*cos(Ar),0.20*sin(Ar),0,Ar,0,0);					
					//Ap.z  += randomGenerator.drawGaussian1D(0,0.05);
					Ap.yaw   += randomGenerator.drawGaussian1D(0,DEG2RAD(0.2));
					Ap.pitch += randomGenerator.drawGaussian1D(0,DEG2RAD(2));
					Ap.roll += randomGenerator.drawGaussian1D(0,DEG2RAD(4));

					incPose3D = CPose3D(Ap);
				}
				else
				{	// 2D path:
					if (circularPath)
					{
						// Circular path:
						float Ar = DEG2RAD(5);
						incPose3D = CPose3D(CPose2D(0.20f*cos(Ar),0.20f*sin(Ar),Ar));
					}
					else
					{
						// Square path:
						if ( (i % squarePathLength) > (squarePathLength-5) )
							incPose3D = CPose3D(CPose2D(0,0,DEG2RAD(90.0f/4)));
						else		incPose3D = CPose3D(CPose2D(0.20f,0,0));
					}
				}
            }
            else
			{
				// Robot is still at the beginning:
				incPose3D = CPose3D(0,0,0,0,0,0);
			}

            // Simulate observations:
            CObservationBearingRangePtr obs=CObservationBearingRange::Create();

            obs->minSensorDistance=minSensorDistance;
            obs->maxSensorDistance=maxSensorDistance;
            obs->fieldOfView_yaw = fieldOfView;
			obs->fieldOfView_pitch = fieldOfView;
            obs->sensorLocationOnRobot = sensorPoseOnRobot;

            landmarkMap.simulateRangeBearingReadings(
                realPose,
                sensorPoseOnRobot,
                *obs,
                sensorDetectsIDs, // wheter to identy landmarks
                stdRange,
                stdYaw,
                stdPitch );

			// Keep the GT of the robot pose:
			GT_path.setSize(i+1,6);
			for (size_t k=0;k<6;k++)
				GT_path(i,k)=realPose[k];

            cout << obs->sensedData.size() << " landmarks in sight";

			if (obs->sensedData.empty()) nWarningsNoSight++;

            SF.push_back( obs );

            // Simulate odometry, from "incPose3D" with noise:
			if (!incPose_is_3D)
			{	// 2D odometry:
				CActionRobotMovement2D    act;
				CPose2D     incOdo( incPose3D );
				if (incPose3D.x()!=0 || incPose3D.y()!=0 || incPose3D.yaw()!=0)
				{
					incOdo.x_incr( randomGenerator.drawGaussian1D_normalized() * odometryNoiseXY_std );
					incOdo.y_incr( randomGenerator.drawGaussian1D_normalized() * odometryNoiseXY_std );
					incOdo.phi_incr( randomGenerator.drawGaussian1D_normalized() * odometryNoisePhi_std );
				}
				act.computeFromOdometry(incOdo, opts);
				acts.insert( act );
			}
			else
			{	// 3D odometry:
				CActionRobotMovement3D    act;
				act.estimationMethod	= CActionRobotMovement3D::emOdometry;
				
				CPose3D   noisyIncPose = incPose3D;

				if (incPose3D.x()!=0 || incPose3D.y()!=0 || incPose3D.yaw()!=0)
				{
					noisyIncPose.x_incr( randomGenerator.drawGaussian1D_normalized() * odometryNoiseXY_std );
					noisyIncPose.y_incr( randomGenerator.drawGaussian1D_normalized() * odometryNoiseXY_std );
					noisyIncPose.z_incr( randomGenerator.drawGaussian1D_normalized() * odometryNoiseXY_std );
					noisyIncPose.setYawPitchRoll(
						noisyIncPose.yaw()+ randomGenerator.drawGaussian1D_normalized() * odometryNoisePhi_std,
						noisyIncPose.pitch()+ randomGenerator.drawGaussian1D_normalized() * odometryNoisePitch_std,
						noisyIncPose.roll()+ randomGenerator.drawGaussian1D_normalized() * odometryNoiseRoll_std );
				}

				act.poseChange.mean = noisyIncPose;
				act.poseChange.cov.eye();

				act.poseChange.cov(0,0) = 
				act.poseChange.cov(1,1) = 
				act.poseChange.cov(2,2) = square(odometryNoiseXY_std);
				act.poseChange.cov(3,3) = square(odometryNoisePhi_std);
				act.poseChange.cov(4,4) = square(odometryNoisePitch_std);
				act.poseChange.cov(5,5) = square(odometryNoiseRoll_std);

				acts.insert( act );
			}

            // Save:
            fil << SF << acts;

            // Next pose:
			realPose = realPose + incPose3D;

            cout << endl;
        }

		// Save the ground truth for the robot poses as well:
		GT_path.saveToTextFile( 
			format("%s/%s_ground_truth_robot_path.txt",outDir.c_str(),outFile.c_str()),
			MATRIX_FORMAT_FIXED,
			true,
			"% Columns are: x(m) y(m) z(m) yaw(rad) pitch(rad) roll(rad)\n");

 		cout << "Data saved to directory: " << outDir << endl;

		if (nWarningsNoSight)
			cout << "WARNING: " << nWarningsNoSight << " observations contained zero landmarks in the sensor range." << endl;


		// Optionally, display in 3D:
		if (show_in_3d && size(GT_path,1)>1)
		{
#if MRPT_HAS_OPENGL_GLUT  && MRPT_HAS_WXWIDGETS
			mrpt::gui::CDisplayWindow3D		win("Final simulation",400,300);

			mrpt::opengl::COpenGLScenePtr &scene = win.get3DSceneAndLock();

			scene->insert( mrpt::opengl::CGridPlaneXY::Create( min_x-10,max_x+10,min_y-10,max_y+10,0 ));
			scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

			// Insert all landmarks:
			for (CLandmarksMap::TCustomSequenceLandmarks::const_iterator it=landmarkMap.landmarks.begin();it!=landmarkMap.landmarks.end();++it)
			{
				mrpt::opengl::CSpherePtr lm = mrpt::opengl::CSphere::Create();
				lm->setColor(1,0,0);
				lm->setRadius(0.1f);
				lm->setLocation( it->pose_mean );
				lm->setName( format("LM#%u",(unsigned) it->ID ) );
				//lm->enableShowName(true);
				scene->insert(lm);
			}

			// Insert all robot poses:
			const size_t N = size(GT_path,1);
			mrpt::opengl::CSetOfLinesPtr  pathLines = mrpt::opengl::CSetOfLines::Create();
			pathLines->setColor(0,0,1,0.5);
			pathLines->setLineWidth(3.0);
			pathLines->resize(N-1);

			for (size_t i=0;i<N-1;i++)
				pathLines->setLineByIndex(i, GT_path(i,0),GT_path(i,1),GT_path(i,2),  GT_path(i+1,0),GT_path(i+1,1),GT_path(i+1,2) );

			scene->insert(pathLines);
			
			for (size_t i=0;i<N;i++)
			{
				mrpt::opengl::CSetOfObjectsPtr  corner = mrpt::opengl::stock_objects::CornerXYZ();
				corner->setScale(0.2f);
				corner->setPose(TPose3D(GT_path(i,0),GT_path(i,1),GT_path(i,2),GT_path(i,3),GT_path(i,4),GT_path(i,5)));
				scene->insert(corner);
			}

			win.unlockAccess3DScene();
			win.forceRepaint();

			cout << "Press any key or close the 3D window to exit." << endl;
			win.waitForKey();

#endif // MRPT_HAS_OPENGL_GLUT  && MRPT_HAS_WXWIDGETS
		}

    }
    catch (std::exception &e)
    {
        std::cout << e.what();
    }
    catch (...)
    {
        std::cout << "Untyped exception!";
    }
}

