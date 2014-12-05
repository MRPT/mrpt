/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


/*-----------------------------------------------------------------------------
	APPLICATION: mex-grabber
	FILE: mexgrabber_main.cpp
	AUTHORS: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>
				Jesus Briales Garcia <jesusbriales@gmail.com>

	For instructions and details, see:
	 http://
  -----------------------------------------------------------------------------*/

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/round.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/slam/CObservationGPS.h>
#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/slam/CSimplePointsMap.h>

// Matlab MEX interface headers
#include <mexplus.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;
using namespace mexplus;

#define CLASS CHokuyoURG

template class mexplus::Session<CLASS>;

namespace {
// Defines MEX API for new.
MEX_DEFINE(new) (int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 1);

    const std::string GLOBAL_SECTION_NAME = "global";

    CGenericSensor::TListObservations		global_list_obs;
    synch::CCriticalSection					cs_global_list_obs;

    bool									allThreadsMustExit = false;

    string 		rawlog_ext_imgs_dir;		// Directory where to save externally stored images, only for CCameraSensor's.

    printf(" rawlog-grabber - Part of the MRPT\n");
    printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
    printf("-------------------------------------------------------------------\n");

    string INI_FILENAME( input.get<string>(0) );
    ASSERT_FILE_EXISTS_(INI_FILENAME)
    CConfigFile	iniFile( INI_FILENAME );
    printf("Using ini file %s\n", INI_FILENAME.c_str());

    // ------------------------------------------
    //			Load config from file:
    // ------------------------------------------
    string			rawlog_prefix = "dataset";
    int				time_between_launches = 300;
    double			SF_max_time_span = 0.25;			// Seconds
    bool			use_sensoryframes = false;
    int				GRABBER_PERIOD_MS = 1000;

    MRPT_LOAD_CONFIG_VAR( rawlog_prefix, string, iniFile, GLOBAL_SECTION_NAME );
    MRPT_LOAD_CONFIG_VAR( time_between_launches, int, iniFile, GLOBAL_SECTION_NAME );
    MRPT_LOAD_CONFIG_VAR( SF_max_time_span, float,		iniFile, GLOBAL_SECTION_NAME );
    MRPT_LOAD_CONFIG_VAR( use_sensoryframes, bool,		iniFile, GLOBAL_SECTION_NAME );
    MRPT_LOAD_CONFIG_VAR( GRABBER_PERIOD_MS, int, iniFile, GLOBAL_SECTION_NAME );

    // Build full rawlog file name:
    string	rawlog_postfix = "_";

    //rawlog_postfix += dateTimeToString( now() );
    mrpt::system::TTimeParts parts;
    mrpt::system::timestampToParts(now(), parts, true);
    rawlog_postfix += format("%04u-%02u-%02u_%02uh%02um%02us",
                             (unsigned int)parts.year,
                             (unsigned int)parts.month,
                             (unsigned int)parts.day,
                             (unsigned int)parts.hour,
                             (unsigned int)parts.minute,
                             (unsigned int)parts.second );

    rawlog_postfix = mrpt::system::fileNameStripInvalidChars( rawlog_postfix );

    // Only set this if we want externally stored images:
    rawlog_ext_imgs_dir = rawlog_prefix+fileNameStripInvalidChars( rawlog_postfix+string("_Images") );

    // Also, set the path in CImage to enable online visualization in a GUI window:
    CImage::IMAGES_PATH_BASE = rawlog_ext_imgs_dir;

    cout << endl ;
    cout << "External image storage: " << rawlog_ext_imgs_dir << endl << endl;

    vector_string	sections;
    iniFile.getAllSections( sections );

    // TODO: Extract sections here
    //string driver_name = params.cfgFile->read_string(params.sensor_label,"driver","",true);

    // Create mexplus handler
    mexPrintf("Before sensor\n");
    //CGenericSensorPtr sensor = CGenericSensor::createSensorPtr("CHokuyoURG");

    CLASS* sensor = new CLASS( );

    //     intptr_t id_ = Session<CGenericSensor>::create( sensor.pointer() );
    //intptr_t id_ = Session<CHokuyoURG>::create( new CHokuyoURG() );
    //intptr_t id_ = Session<CImage>::create( new CImage() );
    //     mexPrintf("Pointer value %d\n", 10);
    //     output.set(0, id_);
    mexPrintf("After sensor pointer creation\n");

    sensor->loadConfig( iniFile, "LASER_2D");
    //sensor->initialize();
    mexPrintf("After sensor initialization\n");

    mrpt::slam::CObservation2DRangeScan	outObservation;
    if(sensor->turnOn())
    {
        printf("Hokuyo correctly initialized\n");
        bool outThereIsObservation;

        bool hardwareError;

        sensor->doProcessSimple(outThereIsObservation,
                                outObservation,
                                hardwareError);
    }

    /*
    mrpt::slam::CObservationImage obs;
    sensor->getObservation( obs );
    obs.image.saveToFile( "/home/jesus/image.jpg" );
    */

    //delete var;
    output.set(0, Session<CLASS>::create( sensor ));

    mexPrintf("Done new, assigning to output\n");
}

//// Defines MEX API for query (const method).
//MEX_DEFINE(read) (int nlhs, mxArray* plhs[],
//                  int nrhs, const mxArray* prhs[]) {
//  InputArguments input(nrhs, prhs, 1);
//  OutputArguments output(nlhs, plhs, 0);

//  const CLASS& sensor = Session<CLASS>::getConst(input.get(0));

//  mrpt::slam::CObservationImage obs;
//  sensor.getObservation( obs );
//  obs.image.saveToFile( "/home/jesus/image.jpg" );

//  //output.set(0, database.query(input.get<string>(1)));
//}

// Defines MEX API for set (non const method).
MEX_DEFINE(read) (int nlhs, mxArray* plhs[],
                  int nrhs, const mxArray* prhs[]) {
  InputArguments input(nrhs, prhs, 1);
  OutputArguments output(nlhs, plhs, 3);
  CLASS* sensor = Session<CLASS>::get(input.get(0));

  bool outThereIsObservation;
  mrpt::slam::CObservation2DRangeScan	outObservation;
  bool hardwareError;

  //sensor->purgeBuffers();
  mrpt::system::sleep(100);
  sensor->doProcessSimple(outThereIsObservation,
                          outObservation,
                          hardwareError);
  //output.set(0, outObservation.scan);

  CSimplePointsMap map;
  map.insertObservation( &outObservation );
  vector<float> xpts;
  vector<float> ypts;
  vector<float> zpts;
  map.getAllPoints(xpts,ypts,zpts);
  printf("Map points recovered\n");

  output.set(0, xpts);
  output.set(1, ypts);
  output.set(2, zpts);

  //database->put(input.get<string>(1), input.get<string>(2));
}

// Defines MEX API for delete.
MEX_DEFINE(delete) (int nlhs, mxArray* plhs[],
                    int nrhs, const mxArray* prhs[]) {
    InputArguments input(nrhs, prhs, 1);
    OutputArguments output(nlhs, plhs, 0);
    Session<CLASS>::destroy(input.get(0));
    //Session<CGenericSensor>::destroy(input.get(0));
    //Session<CHokuyoURG>::destroy(input.get(0));
    //Session<CImage>::destroy(input.get(0));
}

}

MEX_DISPATCH // Don't forget to add this if MEX_DEFINE() is used.
