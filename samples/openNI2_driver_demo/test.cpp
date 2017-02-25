/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/gui.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/opengl.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace std;

//This simple demo records form an OpenNI2 device into a rawlog as 3D
//observations. It's meant as a temporary workaround before OpenNI2 is
//integrated as a generic sensor so that it works with rawlog-grabber.

int main ( int argc, char** argv )
{
	try
	{
		if (argc>2)
		{
			cerr << "Usage: " << argv[0] << " <sensor_id/sensor_serial\n";
			cerr << "Example: " << argv[0] << " 0 \n";
			return 1;
		}

    //const unsigned sensor_id = 0;
    COpenNI2Sensor rgbd_sensor;
//    rgbd_sensor.loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase &configSource,	const std::string	 &iniSection );

    unsigned sensor_id_or_serial = 0;
//		const string configFile = std::string( argv[2] );
		if(argc == 2)
		{
      sensor_id_or_serial = atoi(argv[1]);
		  if(sensor_id_or_serial > 10)
        rgbd_sensor.setSerialToOpen(sensor_id_or_serial);
      else
        rgbd_sensor.setSensorIDToOpen(sensor_id_or_serial);
		}

    // Open:
    //cout << "Calling COpenNI2Sensor::initialize()...";
    rgbd_sensor.initialize();

    if(rgbd_sensor.getNumDevices() == 0)
      return 0;

    cout << "OK " << rgbd_sensor.getNumDevices() << " available devices."  << endl;
    cout << "\nUse device " << sensor_id_or_serial << endl << endl;

    bool showImages = false;
    if(showImages)
    {
      mrpt::gui::CDisplayWindow   win("Video");

      CTicTac	tictac;
      tictac.Tic();
      unsigned int nFrames = 0;
      CObservation3DRangeScan newObs;
      bool bObs = false, bError = true;
      rgbd_sensor.getNextObservation(newObs, bObs, bError);

      while(!system::os::kbhit())
      {
      cout << "Get a new frame\n";
        rgbd_sensor.getNextObservation(newObs, bObs, bError);

        double fps = ++nFrames / tictac.Tac();
  //      newObs->intensityImage.textOut(5,5,format("%.02f fps",fps),TColor(0x80,0x80,0x80) );
        cout << "FPS: " << fps << endl;

        if (nFrames>100)
        {
          tictac.Tic();
          nFrames=0;
        }

        win.showImage(newObs.intensityImage);
        mrpt::system::sleep(10);

      }
    }
    else // Show point cloud and images
    {
      // Create window and prepare OpenGL object in the scene:
      // --------------------------------------------------------
      mrpt::gui::CDisplayWindow3D  win3D("OpenNI2 3D view",800,600);

      win3D.setCameraAzimuthDeg(140);
      win3D.setCameraElevationDeg(20);
      win3D.setCameraZoom(8.0);
      win3D.setFOV(90);
      win3D.setCameraPointingToPoint(2.5,0,0);

      mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
      gl_points->setPointSize(2.5);

      opengl::COpenGLViewportPtr viewInt; // Extra viewports for the RGB images.
      {
        mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

        // Create the Opengl object for the point cloud:
        scene->insert( gl_points );
        scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
        scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

        const double aspect_ratio =  480.0 / 640.0;
        const int VW_WIDTH = 400;	// Size of the viewport into the window, in pixel units.
        const int VW_HEIGHT = aspect_ratio*VW_WIDTH;

        // Create an extra opengl viewport for the RGB image:
        viewInt = scene->createViewport("view2d_int");
        viewInt->setViewportPosition(5, 30, VW_WIDTH,VW_HEIGHT );
        win3D.addTextMessage(10, 30+VW_HEIGHT+10,"Intensity data",TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );

        win3D.addTextMessage(5,5,
          format("'o'/'i'-zoom out/in, ESC: quit"),
            TColorf(0,0,1), 110, MRPT_GLUT_BITMAP_HELVETICA_18 );

        win3D.unlockAccess3DScene();
        win3D.repaint();
      }

      //							Grab frames continuously and show
      //========================================================================================

      bool bObs = false, bError = true;
      mrpt::system::TTimeStamp  last_obs_tim = INVALID_TIMESTAMP;

      while (!win3D.keyHit())	//Push any key to exit // win3D.isOpen()
      {
  //    cout << "Get new observation\n";
        CObservation3DRangeScanPtr newObs = CObservation3DRangeScan::Create();
        rgbd_sensor.getNextObservation(*newObs, bObs, bError);

        if (bObs && !bError && newObs && newObs->timestamp!=INVALID_TIMESTAMP && newObs->timestamp!=last_obs_tim )
        {
          // It IS a new observation:
          last_obs_tim = newObs->timestamp;

          // Update visualization ---------------------------------------

          win3D.get3DSceneAndLock();

          // Estimated grabbing rate:
          win3D.addTextMessage(-350,-13, format("Timestamp: %s", mrpt::system::dateTimeLocalToString(last_obs_tim).c_str()), TColorf(0.6,0.6,0.6),"mono",10,mrpt::opengl::FILL, 100);

          // Show intensity image:
          if (newObs->hasIntensityImage )
          {
            viewInt->setImageView(newObs->intensityImage); // This is not "_fast" since the intensity image may be needed later on.
          }
          win3D.unlockAccess3DScene();

          // -------------------------------------------------------
          //           Create 3D points from RGB+D data
          //
          // There are several methods to do this.
          //  Switch the #if's to select among the options:
          // See also: http://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations
          // -------------------------------------------------------
          if (newObs->hasRangeImage)
          {
    #if 0
            static pcl::PointCloud<pcl::PointXYZRGB> cloud;
            newObs->project3DPointsFromDepthImageInto(cloud, false /* without obs.sensorPose */);

            win3D.get3DSceneAndLock();
              gl_points->loadFromPointsMap(&cloud);
            win3D.unlockAccess3DScene();
    #endif

    // Pathway: RGB+D --> XYZ+RGB opengl
    #if 1
            win3D.get3DSceneAndLock();
              newObs->project3DPointsFromDepthImageInto(*gl_points, false /* without obs.sensorPose */);
            win3D.unlockAccess3DScene();
    #endif
          }

          win3D.repaint();
        } // end update visualization:
      }
    }

    cout << "\nClosing RGBD sensor...\n";

    return 0;

	} catch (std::exception &e)
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


