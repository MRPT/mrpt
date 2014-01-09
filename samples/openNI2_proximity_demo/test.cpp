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
		printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
		openni::OpenNI::shutdown();
		return 1;
	}

	//								Create IR and Depth channels
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


	rc = infrared.create(device, openni::SENSOR_IR);
	if (rc == openni::STATUS_OK)
	{
		rc = infrared.start();
		if (rc != openni::STATUS_OK)
		{
			printf("SimpleViewer: Couldn't start infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
			infrared.destroy();
		}
	}
	else
	{
		printf("SimpleViewer: Couldn't find infrared stream:\n%s\n", openni::OpenNI::getExtendedError());
	}

	if (!depth.isValid() || !infrared.isValid())
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


	//									Read resolution
	//========================================================================================

	options = infrared.getVideoMode();
	printf("\nInitial resolution IR (%d, %d)", options.getResolutionX(), options.getResolutionY());

	options = depth.getVideoMode();
	printf("\nInitial resolution Depth (%d, %d) \n", options.getResolutionX(), options.getResolutionY());


	//										Create scene
	//========================================================================================

	gui::CDisplayWindow3D window;
	opengl::COpenGLScenePtr	scene;
	gui::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1000000;
	window.setWindowTitle("Kinect frame");
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
	const int ir_threshold = 500;
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

		system::sleep(10);
		window.unlockAccess3DScene();
		window.repaint();

	}

	infrared.destroy();
	depth.destroy();
	openni::OpenNI::shutdown();

	return 0;
}


