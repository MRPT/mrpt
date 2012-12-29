/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
