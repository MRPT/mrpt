/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <OpenNI.h>
#include <PS1080.h>

#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/maps.h>

using namespace mrpt;
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
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
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
			printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
			depth.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
	}


	rc = rgb.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK)
	{
		rc = rgb.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
			rgb.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!depth.isValid() || !rgb.isValid())
	{
		printf("SimpleViewer: No valid streams. Exiting\n");
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

	unsigned width = 320, height = 240;

	rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	//rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);

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

    //Allow detection of closer points (although they will flicker)
	//bool CloseRange;
	//depth.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, 1);
	//depth.getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &CloseRange);
	//printf("\nClose range: %s", CloseRange?"On":"Off");	
	

	//										Create scene
	//========================================================================================

	gui::CDisplayWindow3D window;
	opengl::COpenGLScenePtr	scene;
	gui::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1000000;
	window.setWindowTitle("Kinect frame");
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

	slam::CColouredPointsMap points;
	openni::VideoFrameRef framed, framergb;
	
	while (!window.keyHit())	//Push any key to exit
	{
		points.clear();
		depth.readFrame(&framed);
		rgb.readFrame(&framergb);

		if ((framed.getWidth() != framergb.getWidth()) || (framed.getHeight() != framergb.getHeight()))
		{
			cout << endl << "Both frames don't have the same size.";
		}
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
		kinectp->loadFromPointsMap<slam::CColouredPointsMap> (&points); 
		system::sleep(10);
		window.unlockAccess3DScene();
		window.repaint();
	}

	depth.destroy();
	rgb.destroy();
	openni::OpenNI::shutdown();

	return 0;
}


