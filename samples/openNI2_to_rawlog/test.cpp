/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <OpenNI.h>
#include <PS1080.h>

#include <mrpt/base.h>
//#include <mrpt/gui.h>
//#include <mrpt/opengl.h>
#include <mrpt/maps.h>

using namespace mrpt;
using namespace std;

//This simple demo records form an OpenNI2 device into a rawlog as 3D
//observations. It's meant as a temporary workaround before OpenNI2 is
//integrated as a generic sensor so that it works with rawlog-grabber.


int main ( int argc, char** argv )
{
	bool manual_mode = false;
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
	rc = depth.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK)
	{
		rc = depth.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}
	
	
	rc = rgb.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = rgb.start();
		if (rc != openni::STATUS_OK)
		{
			printf("Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
			rgb.destroy();
		}
	}
	else
	{
		printf("Couldn't find infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
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
	
	//						Configure some properties (resolution)
	//========================================================================================
	
	//TODO: 640x480 doesn't work.
	
	const unsigned width = 320, height = 240;
	
	if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	else 
		cout << "Device doesn't do image registration!" << endl;
	
	//	if (device.setDepthColorSyncEnabled(true) == openni::STATUS_OK)
	//		cout << "setDepthColorSyncEnabled" << endl;
	//	else
	//		cout << "setDepthColorSyncEnabled failed!" << endl;
	
	//rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
	

	options = rgb.getVideoMode();
	printf("\nInitial resolution RGB (%d, %d)", options.getResolutionX(), options.getResolutionY());
	options.setResolution(width,height);
	options.setFps(30);
	options.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	rc = rgb.setVideoMode(options);
	if (rc!=openni::STATUS_OK)
	{
		printf("Failed to change RGB resolution!\n");
		return -1;
	}
	rc = rgb.setMirroringEnabled(false);
	
	options = depth.getVideoMode();
	printf("\nInitial resolution Depth(%d, %d)", options.getResolutionX(), options.getResolutionY());
	options.setResolution(width,height);
	options.setFps(30);
	options.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	rc = depth.setVideoMode(options);
	if (rc!=openni::STATUS_OK)
	{
		printf("Failed to change depth resolution!\n");
		return -1;
	}
	rc = depth.setMirroringEnabled(false);
	
	options = depth.getVideoMode();
	printf("\nNew resolution (%d, %d) \n", options.getResolutionX(), options.getResolutionY());
	
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
	slam::CObservation3DRangeScan obs;
	obs.sensorLabel = "KINECT";
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
			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)framed.getData();
			const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)framergb.getData();
			int rowSize = framed.getStrideInBytes() / sizeof(openni::DepthPixel);
			
			obs.timestamp = mrpt::system::getCurrentTime();
			
			obs.hasRangeImage = true;
			obs.rangeImage_forceResetExternalStorage();
			obs.rangeImage_setSize(height,width);
			
			obs.hasIntensityImage = true;
			obs.intensityImageChannel = slam::CObservation3DRangeScan::CH_VISIBLE;
			
			const string sDepthfile = mrpt::format("%i_depth.bin", frame );
			const string sRGBfile = mrpt::format("%i_rgb.png", frame );
			
			utils::CImage iimage(width,height,CH_RGB);
			for (int yc = 0; yc < framed.getHeight(); ++yc)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				const openni::RGB888Pixel* pRgb = pRgbRow;
				for (int xc = 0; xc < framed.getWidth(); ++xc, ++pDepth, ++pRgb)
				{
					obs.rangeImage(yc,xc) = (*pDepth)*1.0/1000;
					iimage.setPixel(xc,yc,(pRgb->r<<16)+(pRgb->g<<8)+pRgb->b);
					
					//obs.intensityImage.setPixel(xc,yc,(*pRgb));
					
				}
				
				pDepthRow += rowSize;
				pRgbRow += rowSize;
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


