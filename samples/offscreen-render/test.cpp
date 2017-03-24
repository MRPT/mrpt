/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/opengl/CFBORender.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CImage.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
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
