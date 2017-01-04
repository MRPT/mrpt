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

#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/system/threads.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace std;


int main ( int argc, char** argv )
{
	openni::Status rc = openni::STATUS_OK;
	openni::Device		device;
	openni::VideoMode	options;
	openni::VideoStream depth, rgb;

	//									Device is openned
	//=======================================================================================
	const char* deviceURI = openni::ANY_DEVICE;
	if (argc > 1)
		deviceURI = argv[1];

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

	//							Create RGB and Depth channels
	//========================================================================================
	rc = depth.create(device, openni::SENSOR_DEPTH);
	rc = rgb.create(device, openni::SENSOR_COLOR);


	//							Configure video properties
	//========================================================================================
	int width = 640, height = 480;

	rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	//rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
	
	openni::VideoMode vm;

	options = rgb.getVideoMode();
	printf("\nInitial resolution RGB (%d, %d)", options.getResolutionX(), options.getResolutionY());
	options.setResolution(width,height);
	rc = rgb.setVideoMode(options);
	rc = rgb.setMirroringEnabled(false);

	options = depth.getVideoMode();
	printf("\nInitial resolution Depth(%d, %d)", options.getResolutionX(), options.getResolutionY());
	options.setResolution(width,height);
	rc = depth.setVideoMode(options);
	rc = depth.setMirroringEnabled(false);

	options = depth.getVideoMode();
	printf("\nNew resolution (%d, %d) \n", options.getResolutionX(), options.getResolutionY());


	rc = depth.start();
	if (rc != openni::STATUS_OK)
	{
		printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		depth.destroy();
	}

	rc = rgb.start();
	if (rc != openni::STATUS_OK)
	{
		printf("Couldn't start rgb stream:\n%s\n", openni::OpenNI::getExtendedError());
		rgb.destroy();
	}

	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return 3;
	}

	if (!depth.isValid() || !rgb.isValid())
	{
		printf("No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	//						Uncomment this to see the video modes available
	//========================================================================================
	////Depth modes
	//for(unsigned int i = 0; i<depth.getSensorInfo().getSupportedVideoModes().getSize(); i++)
	//{
	//	vm = depth.getSensorInfo().getSupportedVideoModes()[i];
	//	printf("\n Depth mode %d: %d x %d, fps - %d Hz, pixel format - ",i, vm.getResolutionX(), vm.getResolutionY(), vm.getFps());
	//	cout << vm.getPixelFormat();
	//	if ((vm.getResolutionX() == width)&&(vm.getPixelFormat() == openni::PIXEL_FORMAT_DEPTH_1_MM))
	//		rc = depth.setVideoMode(vm);
	//}
	
	////Colour modes
	//for(unsigned int i = 0; i<rgb.getSensorInfo().getSupportedVideoModes().getSize(); i++)
	//{
	//	vm = rgb.getSensorInfo().getSupportedVideoModes()[i];
	//	printf("\n RGB mode %d: %d x %d, fps - %d Hz, pixel format - ",i, vm.getResolutionX(), vm.getResolutionY(), vm.getFps());
	//	cout << vm.getPixelFormat();
	//}

    //					Uncomment this to allow for closer points detection
	//========================================================================================
	//bool CloseRange;
	//depth.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);
	//depth.getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &CloseRange);
	//printf("\nClose range: %s", CloseRange?"On":"Off");	
	

	//										Create scene
	//========================================================================================
	gui::CDisplayWindow3D window;
	opengl::COpenGLScenePtr	scene;
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1000000;
	window.setWindowTitle("RGB-D camera frame");
	window.resize(800,600);
	window.setPos(500,50);
	window.setCameraZoom(5);
	window.setCameraAzimuthDeg(180);
	window.setCameraElevationDeg(5);
	scene = window.get3DSceneAndLock();

	opengl::CPointCloudColouredPtr kinectp = opengl::CPointCloudColoured::Create();
	kinectp->enablePointSmooth(true);
	kinectp->setPointSize(2);
	scene->insert( kinectp );

	opengl::CSetOfObjectsPtr reference = opengl::stock_objects::CornerXYZ();
	reference->setScale(0.4);
	scene->insert( reference );

	window.unlockAccess3DScene();
	window.addTextMessage(5,5, format("Push any key to exit"));
	window.repaint();

	//							Grab frames continuously and show
	//========================================================================================
	CColouredPointsMap points;
	openni::VideoFrameRef framed, framergb;
	
	while (!window.keyHit())	//Push any key to exit
	{
		points.clear();
		depth.readFrame(&framed);
		rgb.readFrame(&framergb);

		if ((framed.getWidth() != framergb.getWidth()) || (framed.getHeight() != framergb.getHeight()))
			cout << endl << "Both frames don't have the same size.";

		else
		{
			//Read one frame
			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)framed.getData();
			const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)framergb.getData();
			int rowSize = framed.getStrideInBytes() / sizeof(openni::DepthPixel);

			float x, y, z, inv_f = float(640/width)/525.0f;

			for (int yc = 0; yc < framed.getHeight(); ++yc)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				const openni::RGB888Pixel* pRgb = pRgbRow;
				for (int xc = 0; xc < framed.getWidth(); ++xc, ++pDepth, ++pRgb)
				{
					z = 0.001*(*pDepth);
					x = -(xc - 0.5*(width-1))*z*inv_f;
					y = -(yc - 0.5*(height-1))*z*inv_f;
					points.insertPoint(z, x, y, 0.0039*pRgb->r, 0.0039*pRgb->g, 0.0039*pRgb->b);
				}
				pDepthRow += rowSize;
				pRgbRow += rowSize;
			}
		}

		scene = window.get3DSceneAndLock();
		kinectp->loadFromPointsMap<mrpt::maps::CColouredPointsMap> (&points);
		system::sleep(5);
		window.unlockAccess3DScene();
		window.repaint();
	}

	depth.destroy();
	rgb.destroy();
	openni::OpenNI::shutdown();

	return 0;
}


