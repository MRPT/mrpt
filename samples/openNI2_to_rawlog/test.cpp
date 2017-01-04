/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// This seems to be assumed by OpenNI.h and undefined for some reason in GCC/Ubuntu
#if !defined(MRPT_OS_WINDOWS)
#   define linux 1
#endif
#include <OpenNI.h>
#include <PS1080.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl.h>
#include <mrpt/obs/CObservation3DRangeScan.h>


using namespace mrpt;
using namespace std;

//This simple demo records form an OpenNI2 device into a rawlog as 3D
//observations. It's meant as a temporary workaround before OpenNI2 is
//integrated as a generic sensor so that it works with rawlog-grabber.

bool setONI2StreamMode  (openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format);
bool initONI2Stream(openni::Device& device, openni::SensorType sensorType, openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format);
bool synchONI2MirrorMode(openni::VideoStream& rgb, openni::VideoStream& depth, bool& isMirrorMode);
bool startONI2Streams(openni::VideoStream& rgb, openni::VideoStream& depth);

int main ( int argc, char** argv )
{
	bool manual_mode = false;
	//						Properties
	//========================================================================================
	const unsigned width = 640, height = 480, fps = 30;

	try
	{
		if (argc<2)
		{
			cerr << "Usage: " << argv[0] << " OUTPUT_PATH [-m]\n";
			cerr << "Example: " << argv[0] << " /home/jdoe/myrawlog will produce file myrawlog.rawlog in folder /home/jdoe\n";
			cerr << "-m will activate manual mode\n";
			return 1;
		}
		else if ((argc == 3) &&	(string(argv[2]) == "-m"))
			manual_mode = true;

	openni::Status rc = openni::STATUS_OK;

	openni::Device		device;
	openni::VideoMode	options;
	openni::VideoStream depth, rgb;

	//									Device is opened
	//=======================================================================================
	const char* deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();

	if (rc != openni::STATUS_OK) { printf("After initialization:\n %s\n", openni::OpenNI::getExtendedError()); }

	rc = device.open(deviceURI);

	//cout << endl << "Do we have IR sensor? " << device.hasSensor(openni::SENSOR_IR);
	//cout << endl << "Do we have RGB sensor? " << device.hasSensor(openni::SENSOR_COLOR);
	//cout << endl << "Do we have Depth sensor? " << device.hasSensor(openni::SENSOR_DEPTH);

	if (rc != openni::STATUS_OK)
	{
		printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	//								Create RGB and Depth channels
	//========================================================================================
    if(initONI2Stream(device, openni::SENSOR_COLOR, rgb,   width, height, fps, openni::PIXEL_FORMAT_RGB888) == false){
      return 1;
    }
    if(initONI2Stream(device, openni::SENSOR_DEPTH, depth, width, height, fps, openni::PIXEL_FORMAT_DEPTH_1_MM) == false){
      return 1;
    }
    bool isMirror;
    if(synchONI2MirrorMode(rgb, depth, isMirror) == false){
      return 1;
    }
    if(startONI2Streams(rgb, depth) == false){
      return 1;
    }
	if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)){
		rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}else{
		cout << "Device doesn't do image registration!" << endl;
	}

	if (!depth.isValid() || !rgb.isValid())
	{
		printf("No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return 3;
	}

	//	if (device.setDepthColorSyncEnabled(true) == openni::STATUS_OK)
	//		cout << "setDepthColorSyncEnabled" << endl;
	//	else
	//		cout << "setDepthColorSyncEnabled failed!" << endl;

	//rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);

	//Allow detection of closer points (although they will flicker)
	//bool CloseRange;
	//depth.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);
	//depth.getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &CloseRange);
	//printf("\nClose range: %s", CloseRange?"On":"Off");


	//							Output files setup
	//=================depth=======================================================================
	//slam::CColouredPointsMap points;
	openni::VideoFrameRef framed, framergb;

	const string out_name = string(argv[1]);

	// Create output directory for images ------------------------------
	const string  out_img_dir = out_name + string("_Images");

	cout << "Creating images directory: " << out_img_dir << endl;
	mrpt::system::createDirectory(out_img_dir);

	// Create rawlog file ----------------------------------------------
	const string  out_rawlog_fil = out_name + string(".rawlog");
	cout << "Creating rawlog: " << out_rawlog_fil << endl;
	mrpt::utils::CFileGZOutputStream  f_out(out_rawlog_fil);

	// Fill out the common field to all entries:
	mrpt::obs::CObservation3DRangeScan obs;
	obs.sensorLabel = "OpenNI2";
	obs.hasConfidenceImage = false;

	// Kinect style: ranges are actually depth values, not Euclidean distances.
	obs.range_is_depth = true;

	// Calculate depth camera parameters (from pcl WIP)
	float hFov = depth.getHorizontalFieldOfView();
	float fx = width / (2.0f * tan (hFov / 2.0f));
	float vFov = depth.getVerticalFieldOfView();
	float fy = height / (2.0f * tan (vFov / 2.0f));

	printf("Got Fov from the device, calculated fx and fy:\n"
		   "hFov:%f\n"
		   "vFov:%f\n"
		   "fx:%f\n"
		   "fy:%f\n",
		   hFov,
		   vFov,
		   fx,
		   fy);

	obs.cameraParams.nrows = width;
	obs.cameraParams.ncols = height;
	obs.cameraParams.fx(fx);
	obs.cameraParams.fy(fy);
	obs.cameraParams.cx(( width-1)*0.5);
	obs.cameraParams.cy((height-1)*0.5);
	obs.cameraParamsIntensity = obs.cameraParams;

	// Images are already registered from OpenNI2
	obs.relativePoseIntensityWRTDepth = mrpt::poses::CPose3D(0,0,0,0,0,0);
	obs.hasPoints3D = false;


	//							Grab frames and record
	//========================================================================================

	utils::CTicTac  tictac;
	unsigned int frame = 0;

	bool depthfirst = true;
	bool sampleinsync = false;

	if (manual_mode)
		cout << "Press any key to grab a frame. 'ESC' to stop\n";
	else
		cout << "Press any key to stop\n";

	while ((!system::os::kbhit() && !manual_mode) || manual_mode)
	{
		if (manual_mode)
		{
			if (system::os::getch()==27)
				break;
		}

		frame ++;

		//This loop makes sure our samples of depth and RGB are in sync.
		//It should only iterate once if samples are being recorded out of
		//sync.
		sampleinsync = false;
		while(!sampleinsync)
		{
			tictac.Tic();
			if(depthfirst)
				depth.readFrame(&framed);
			else
				rgb.readFrame(&framergb);
			double t1 = tictac.Tac()*1000;

			tictac.Tic();
			if(depthfirst)
				rgb.readFrame(&framergb);
			else
				depth.readFrame(&framed);

			double t2 = tictac.Tac()*1000;
			cout << "frame " << frame <<". t1: " << t1 << ", t2: " << t2;

			if (t2>t1)
			{
				cout << " --- out of sync";
				depthfirst = !depthfirst;
			}
			else
			{
				cout << " --- in sync    ";
				sampleinsync=true;
			}

			cout << "\r";
		}


		if ((framed.getWidth() != framergb.getWidth()) || (framed.getHeight() != framergb.getHeight()))
		{
			cout << "\nBoth frames don't have the same size.";
		}
		else
		{
			// Read one frame
			const char* pDepthRow = (const char*)framed.getData();
			const char* pRgbRow   = (const char*)framergb.getData();
			int d_step   = framed.getStrideInBytes();
			int rgb_step = framergb.getStrideInBytes();

			obs.timestamp = mrpt::system::getCurrentTime();

			obs.hasRangeImage = true;
			obs.rangeImage_forceResetExternalStorage();
			obs.rangeImage_setSize(height,width);

			obs.hasIntensityImage = true;
			obs.intensityImageChannel = mrpt::obs::CObservation3DRangeScan::CH_VISIBLE;

			const string sDepthfile = mrpt::format("%i_depth.bin", frame );
			const string sRGBfile = mrpt::format("%i_rgb.png", frame );

			utils::CImage iimage(width,height,CH_RGB);
			for (unsigned int yc = 0; yc < height; ++yc)
			{
				const openni::DepthPixel*  pDepth = (const openni::DepthPixel* )pDepthRow;
				const openni::RGB888Pixel* pRgb   = (const openni::RGB888Pixel*)pRgbRow;
				for (unsigned int xc = 0; xc < width; ++xc, ++pDepth, ++pRgb)
				{
					int _x = xc;
					if(isMirror){
					  _x = width -_x - 1;
					}
					obs.rangeImage (yc, _x) = (*pDepth)*1.0/1000;
					iimage.setPixel(_x, yc, (pRgb->r<<16)+(pRgb->g<<8)+pRgb->b);

					//obs.intensityImage.setPixel(xc,yc,(*pRgb));

				}
				pDepthRow += d_step;
				pRgbRow   += rgb_step;
			}
			obs.rangeImage_convertToExternalStorage( sDepthfile, out_img_dir + string("/") );
			obs.intensityImage = iimage;
			obs.intensityImage.saveToFile( out_img_dir + string("/") + sRGBfile );
			obs.intensityImage.setExternalStorage(out_img_dir + string("/") + sRGBfile);

			f_out << obs;
		}

	}

	cout << "\nStopping...\n";

	depth.destroy();
	rgb.destroy();
	openni::OpenNI::shutdown();

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




bool setONI2StreamMode(openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format){
	/*
  void openni::VideoMode::setResolution()
	Setter function for the resolution of this VideoMode. Application use of this function is not recommended.
	Instead, use SensorInfo::getSupportedVideoModes() to obtain a list of valid video modes

  -- cited from OpenNI2 help. setResolution() is not recommended.
  */
	bool found = false;
	const openni::Array<openni::VideoMode>& modes = stream.getSensorInfo().getSupportedVideoModes();
	for(int i = 0, i_end = modes.getSize();i < i_end;++i){
		if(modes[i].getResolutionX() != w){
			continue;
		}
		if(modes[i].getResolutionY() != h){
			continue;
		}
		if(modes[i].getPixelFormat() != format){
			continue;
		}
		openni::Status rc = stream.setVideoMode(modes[i]);
		if(rc != openni::STATUS_OK){
			printf("%s:Couldn't find RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
			return false;
		}
		return true;
	}
	return false;
}

#if 0
bool initONI2RGBStream(openni::Device& device, openni::VideoStream& rgb, int w, int h, int fps, openni::PixelFormat format){
	openni::Status rc = openni::STATUS_OK;
	rc = rgb.create(device, openni::SENSOR_COLOR);
	if(rc != openni::STATUS_OK){
		printf("%s:Couldn't find RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	rc = rgb.setMirroringEnabled(false);
	if (rc != openni::STATUS_OK){
		printf("%s:setMirroringEnabled(false) failed:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	openni::VideoMode options = rgb.getVideoMode();
	printf("Initial resolution RGB (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	if(setONI2StreamMode(rgb, w, h, fps, format) == false){
		printf("%s:Can't find desired rgb mode\n", __FUNCTION__ );
		return false;
	}
	options = rgb.getVideoMode();
	printf("  -> (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	rc = rgb.start();
	if (rc != openni::STATUS_OK){
		printf("%s:Couldn't start RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		rgb.destroy();
		return false;
	}
	return true;
}

bool initONI2DepthStream(openni::Device& device, openni::VideoStream& depth, int w, int h, int fps, openni::PixelFormat format){
	openni::Status rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc != openni::STATUS_OK){
		printf("%s:Couldn't find depth stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	openni::VideoMode options = depth.getVideoMode();
	printf("Initial resolution Depth(%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	if(setONI2StreamMode(depth, w, h, fps, format) == false){
		printf("%s:Can't find desired depth mode\n", __FUNCTION__ );
		return false;
	}
	options = depth.getVideoMode();
	printf("  -> (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	depth.setMirroringEnabled(false);
	rc = depth.start();
	if (rc != openni::STATUS_OK){
		printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		depth.destroy();
		return false;
	}
	return true;
}
#endif

bool initONI2Stream(openni::Device& device, openni::SensorType sensorType, openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format){
  openni::Status rc = openni::STATUS_OK;
  const char* strSensor;
  if(sensorType == openni::SENSOR_COLOR){
    strSensor = "openni::SENSOR_COLOR";
  }else if(sensorType == openni::SENSOR_DEPTH){
    strSensor = "openni::SENSOR_DEPTH";
  }else{
	printf("%s:Unknown SensorType -> %d\n", __FUNCTION__, sensorType);
    return false;
  }
  rc = stream.create(device, sensorType);
  if(rc != openni::STATUS_OK){
    printf("%s:Couldn't find sensor %s: %s\n", __FUNCTION__, strSensor, openni::OpenNI::getExtendedError());
    return false;
  }
  openni::VideoMode options = stream.getVideoMode();
  printf("%s:Initial resolution %s (%d, %d) FPS %d Format %d\n", __FUNCTION__, strSensor, options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
  if(setONI2StreamMode(stream, w, h, fps, format) == false){
    printf("%s:Can't find desired mode in the %s\n", __FUNCTION__, strSensor);
    return false;
  }
  options = stream.getVideoMode();
  printf("  -> (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
  return true;
}

bool synchONI2MirrorMode(openni::VideoStream& rgb, openni::VideoStream& depth, bool& isMirrorMode){
  static const int SIZE = 2;
  openni::VideoStream* streams[SIZE]  = { &rgb,                   &depth                 };
  static const char*   strNames[SIZE] = { "openni::SENSOR_COLOR", "openni::SENSOR_DEPTH" };
  isMirrorMode       = false;
  // Check whether both stream support mirroring.
  for(int i = 0;i < SIZE;++i){
    if(streams[i]->isPropertySupported(openni::STREAM_PROPERTY_MIRRORING) == false){
      printf("%s:openni::STREAM_PROPERTY_MIRRORING is not supported on %s.\n"
        "  We assume this is MS Kinect and taken images are inverted to right and left.\n", __FUNCTION__, strNames[i]);
      // In this case, getMirroringEnabled() method always returns false. So we cannot confirm whether the images are inverted or not.
      isMirrorMode  = true;
      break;
    }
  }
  // Set both stream to same mirror mode.
  for(int i = 0;i < SIZE;++i){
    if(streams[i]->isPropertySupported(openni::STREAM_PROPERTY_MIRRORING) == false){
      break;
    }
    if(streams[i]->setMirroringEnabled(isMirrorMode) != openni::STATUS_OK){
      printf("%s:setMirroringEnabled() failed: %s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
      return false;
    }
  }
  return true;
}

bool startONI2Streams(openni::VideoStream& rgb, openni::VideoStream& depth){ 
  static const int SIZE = 2;
  openni::VideoStream* streams[SIZE]  = { &rgb,                   &depth                 };
  static const char*   strNames[SIZE] = { "openni::SENSOR_COLOR", "openni::SENSOR_DEPTH" };
  for(int i = 0;i < SIZE;++i){
    openni::Status rc = streams[i]->start();
    if (rc != openni::STATUS_OK){
      printf("%s:Couldn't start RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
      streams[i]->destroy();
      return false;
    }
  }
  return true;
}
