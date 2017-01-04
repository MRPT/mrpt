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
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::random;
using namespace std;


int main(int argc, char ** argv)
{
    try
    {
		bool showHelp    = argc>1 && !os::_strcmp(argv[1],"--help");
		bool showVersion = argc>1 && !os::_strcmp(argv[1],"--version");

		printf(" simul-beacons - Part of the MRPT\n");
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

        randomGenerator.randomize();

        int  	i;
        char    auxStr[2000];

	// Set default values:
	int 		nBeacons = 3;
        int		nSteps = 100;
        std::string 	outFile("out.rawlog");
        std::string 	outDir("OUT");

        float min_x =-5;
        float max_x =5;
        float min_y =-5;
        float max_y =5;
        float min_z =0;
        float max_z =0;

        float odometryNoiseXY_std = 0.001f;
        float odometryBias_Y = 0;

        float minSensorDistance = 0;
        float maxSensorDistance = 10;
        float stdError = 0.04f;
	bool  circularPath = true;
	int   squarePathLength=40;


		float ratio_outliers = 0;
		float ratio_outliers_first_step = 0;
		float outlier_uniform_min=0;
		float outlier_uniform_max=5.0;

	// Load params from INI:
	MRPT_LOAD_CONFIG_VAR(nBeacons,int,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(outFile,string,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(outDir,string,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(nSteps,int,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(circularPath,bool,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(squarePathLength,int,ini,"Params");


	MRPT_LOAD_CONFIG_VAR(ratio_outliers,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(ratio_outliers_first_step,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(outlier_uniform_min,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(outlier_uniform_max,float,	ini,"Params");


	MRPT_LOAD_CONFIG_VAR(min_x,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(max_x,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(min_y,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(max_y,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(min_z,float,	ini,"Params");

	MRPT_LOAD_CONFIG_VAR(odometryNoiseXY_std,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(odometryBias_Y,float,	ini,"Params");

	MRPT_LOAD_CONFIG_VAR(minSensorDistance,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(maxSensorDistance,float,	ini,"Params");
	MRPT_LOAD_CONFIG_VAR(stdError,float,	ini,"Params");

        // Create out dir:
        mrpt::system::createDirectory(outDir);

        // ---------------------------------------------
        // Create the point-beacons:
        // ---------------------------------------------
        printf("Creating beacon map...");
		mrpt::maps::CBeaconMap    beaconMap;
        for (i=0;i<nBeacons;i++)
        {
            CBeacon     b;
            CPoint3D    pt3D;

            // Random coordinates:
            pt3D.x( randomGenerator.drawUniform(min_x,max_x) );
            pt3D.y( randomGenerator.drawUniform(min_y,max_y) );
            pt3D.z( randomGenerator.drawUniform(min_z,max_z) );

            // Add:
            b.m_typePDF=CBeacon::pdfMonteCarlo;
            b.m_locationMC.setSize(1,pt3D);
			b.m_ID = i;
            beaconMap.push_back(b);
        }


        os::sprintf(auxStr,sizeof(auxStr),"%s/outSimMap.txt",outDir.c_str());
        beaconMap.saveToTextFile(auxStr);
        printf("Done!\n");

        //beaconMap.simulateBeaconReadings(  );

        // ---------------------------------------------
        // Simulate:
        // ---------------------------------------------
        CActionRobotMovement2D::TMotionModelOptions   opts;
        CPoint3D                null3D(0,0,0);
        opts.modelSelection = CActionRobotMovement2D::mmGaussian;
	opts.gaussianModel.a1=0;
	opts.gaussianModel.a2=0;
	opts.gaussianModel.a3=0;
	opts.gaussianModel.a4=0;
	opts.gaussianModel.minStdXY = odometryNoiseXY_std;
	opts.gaussianModel.minStdPHI = DEG2RAD( 0.002f );

        os::sprintf(auxStr,sizeof(auxStr),"%s/%s",outDir.c_str(),outFile.c_str());
        CFileOutputStream     fil(auxStr);
        CPose2D         realPose;
	CPose2D         incPose;
	int	stopSteps = 4;

        for (i=0;i<nSteps;i++)
        {
            printf("Generating step %i...",i);
            CSensoryFrame           SF;
            CActionCollection         acts;
            CActionRobotMovement2D    act;
            CPose3D                 pose3D( realPose );


			if (i>=stopSteps)
			{
				if (circularPath)
				{
					// Circular path:
					float Ar = DEG2RAD(5);
					incPose = CPose2D(0.20f*cos(Ar),0.20f*sin(Ar),Ar);
				}
				else
				{
					// Square path:
					if ( (i % squarePathLength) > (squarePathLength-5) )
							incPose = CPose2D(0,0,DEG2RAD(90.0f/4));
					else		incPose = CPose2D(0.20f,0,0);
				}
			}
			else	incPose = CPose2D(0,0,0);

            // Simulate observations:
            CObservationBeaconRangesPtr obs=CObservationBeaconRanges::Create();
            obs->minSensorDistance=minSensorDistance;
            obs->maxSensorDistance=maxSensorDistance;
            obs->stdError=stdError;

            beaconMap.simulateBeaconReadings( pose3D,null3D,*obs );

            // Corrupt with ourliers:
            float probability_corrupt = i==0 ? ratio_outliers_first_step : ratio_outliers;
            for (size_t q=0;q<obs->sensedData.size();q++)
            {
            	if ( randomGenerator.drawUniform(0.0f,1.0f) < probability_corrupt )
            	{
					obs->sensedData[q].sensedDistance += randomGenerator.drawUniform( outlier_uniform_min,outlier_uniform_max );
					if (obs->sensedData[q].sensedDistance<0)
						obs->sensedData[q].sensedDistance = 0;
            	}
            }

			std::cout << obs->sensedData.size() << " beacons at range";

            SF.push_back( obs );

            // Simulate actions:
            CPose2D     incOdo( incPose );
            if (incPose.x()!=0 || incPose.y()!=0 || incPose.phi()!=0)
			{
	            incOdo.x_incr( randomGenerator.drawGaussian1D_normalized() * odometryNoiseXY_std );
        	    incOdo.y_incr( odometryBias_Y + randomGenerator.drawGaussian1D_normalized() * odometryNoiseXY_std );
			}
            act.computeFromOdometry( incOdo, opts);
            acts.insert( act );

            // Save:
            fil << SF << acts;

            // Next pose:
            realPose = realPose + incPose;

	    printf("\n");
        }


		cout << "Data saved to directory: " << outDir << endl;
    }
    catch(std::exception &e)
    {
        std::cout << e.what();
    }
    catch(...)
    {
        std::cout << "Untyped exception!";
    }
}

