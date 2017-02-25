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

#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/maps.h>

using namespace mrpt;
using namespace std;

//=================================================================================================
//
// This example illustrates how points closer than a RGBD camera threshold (0.3 - 0.5 m typically)
// can be detected with these cameras. This method finds pixels with a very high value in the
// infrared image and, therefore, it will fail if an external source of infrared light (sun) affects
// the camera...
//
// =================================================================================================


int main ( int argc, char** argv )
{
	openni::Status rc = openni::STATUS_OK;
	openni::Device		device;
	openni::VideoMode	options;
	openni::VideoStream depth, infrared;
	
	//									Device is openned
	//=======================================================================================
	const char* deviceURI = openni::ANY_DEVICE;
	if (argc > 1)
		deviceURI = argv[1];

	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK) { printf("After initialization:\n %s\n", openni::OpenNI::getExtendedError()); }
	rc = device.open(deviceURI);

	if (rc != openni::STATUS_OK)
	{
		printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	//								Create IR and Depth channels
	//========================================================================================
	rc = depth.create(device, openni::SENSOR_DEPTH);
	rc = infrared.create(device, openni::SENSOR_IR);

	//							Read resolution and start streams
	//========================================================================================
	options = infrared.getVideoMode();
	printf("\nInitial resolution IR (%d, %d)", options.getResolutionX(), options.getResolutionY());
	options = depth.getVideoMode();
	printf("\nInitial resolution Depth (%d, %d) \n", options.getResolutionX(), options.getResolutionY());

	rc = depth.start();
	if (rc != openni::STATUS_OK)
	{
		printf("Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		depth.destroy();
	}

	rc = infrared.start();
	if (rc != openni::STATUS_OK)
	{
		printf("Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
		infrared.destroy();
	}

	if (rc != openni::STATUS_OK)
	{
		openni::OpenNI::shutdown();
		return 3;
	}

	if (!depth.isValid() || !infrared.isValid())
	{
		printf("No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}

	//						Uncomment this to see the video modes available
	//========================================================================================
	//Infrared modes
	//openni::VideoMode vm;
	//for(unsigned int i = 0; i<infrared.getSensorInfo().getSupportedVideoModes().getSize(); i++)
	//{
	//	vm = infrared.getSensorInfo().getSupportedVideoModes()[i];
	//	printf("\n Depth mode %d: %d x %d, fps - %d Hz, pixel format - ",i, vm.getResolutionX(), vm.getResolutionY(), vm.getFps());
	//	cout << vm.getPixelFormat();
	//}


	//										Create scene
	//========================================================================================
	gui::CDisplayWindow3D window;
	opengl::COpenGLScenePtr	scene;
	mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1000000;
	window.setWindowTitle("RGB-D camera frame");
	window.resize(800,600);
	window.setPos(500,50);
	window.setCameraZoom(5);
	window.setCameraAzimuthDeg(-90);
	window.setCameraElevationDeg(90);
	scene = window.get3DSceneAndLock();

	opengl::CSetOfLinesPtr lines = opengl::CSetOfLines::Create();
	lines->setLocation(0,0,0);
	lines->setColor(0,0,1);
	lines->setLineWidth(10);
	lines->appendLine(-1,1,0,1,1,0);
	lines->appendLine(1,1,0,1,-1,0);
	lines->appendLine(1,-1,0,-1,-1,0);
	lines->appendLine(-1,-1,0,-1,1,0);
	scene->insert( lines );

	opengl::CPointCloudPtr too_close = opengl::CPointCloud::Create();
	too_close->setPointSize(5);
	too_close->enablePointSmooth(true);
	too_close->setColor(1,0,0);
	scene->insert( too_close );

	opengl::CGridPlaneXYPtr grid = opengl::CGridPlaneXY::Create(-4,4,-4,4,0);
	scene->insert( grid );

	window.unlockAccess3DScene();
	window.addTextMessage(5,5, format("Push any key to exit"));
	window.repaint();


	//					Grab frames continuously and show saturated pixels
	//========================================================================================

	openni::VideoFrameRef framed, frameir;
	const int ir_threshold = 700;	//It can vary depending on the camera and the environment
	float x,y;

	while (!window.keyHit())	//Push any key to exit
	{
		infrared.readFrame(&frameir);
		depth.readFrame(&framed);

		scene = window.get3DSceneAndLock();	
		too_close->clear();

		//Read one frame
		if ((framed.getWidth() != frameir.getWidth()) || (framed.getHeight() != frameir.getHeight()))
		{
			cout << endl << "Both frames don't have the same size.";
		}
		else
		{
			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)framed.getData();
			const openni::Grayscale16Pixel* pInfraredRow = (const openni::Grayscale16Pixel*)frameir.getData();
			int rowSize = frameir.getStrideInBytes() / sizeof(openni::Grayscale16Pixel);

			for (int yc = 0; yc < frameir.getHeight(); ++yc)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				const openni::Grayscale16Pixel* pInfrared = pInfraredRow;
				for (int xc = 0; xc < frameir.getWidth(); ++xc, ++pInfrared, ++pDepth)
				{
					
					if ((*pInfrared > ir_threshold)&&(*pDepth == 0))
					{
						x = -1.0f + float(2*xc)/float(frameir.getWidth());
						y = -1.0f + float(2*yc)/float(frameir.getHeight());
						too_close->insertPoint( x, -y, 0);
					}
				}
				
				pDepthRow += rowSize;
				pInfraredRow += rowSize;
			}
		}

		system::sleep(5);
		window.unlockAccess3DScene();
		window.repaint();

	}

	infrared.destroy();
	depth.destroy();
	openni::OpenNI::shutdown();

	return 0;
}


