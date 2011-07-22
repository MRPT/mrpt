/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
#include <mrpt/opengl.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

// ------------------------------------------------------
//				TestOffscreenRender
// ------------------------------------------------------
int TestOffscreenRender(int argc,char *argv[])
{
	if (argc!=5)
	{
		cerr << "Usage: " << argv[0] << " <FILE.3Dscene> <WIDTH> <HEIGHT> <OUTPUT_IMAGE>\n"
		    " Output image can be in many formats: .png, .jpg, .tif, etc.\n";
		return 1;
	}

	const string sFil = argv[1];
	const int width = atoi(argv[2]);
	const int height = atoi(argv[3]);
	const string sOut = argv[4];

	ASSERT_ABOVE_(width,0)
	ASSERT_ABOVE_(height,0)

	COpenGLScene scene;
	CFileGZInputStream f(sFil);
	f >> scene;

	CFBORender render(width, height);
	CImage frame(width, height, 3, false);

	// here you can put your preferred camera rendering position
	{
		CCamera& camera = render.getCamera(scene);
		camera.setOrthogonal(true);
		camera.setZoomDistance(70);
		camera.setAzimuthDegrees(-90);
		camera.setElevationDegrees(90);
	}

	// render the scene
	render.getFrame2(scene, frame);
	frame.saveToFile(sOut);

	return 0;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char* argv[])
{
	try
	{
		return TestOffscreenRender(argc,argv);
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
