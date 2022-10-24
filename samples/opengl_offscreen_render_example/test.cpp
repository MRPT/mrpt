/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/img/CImage.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/serialization/CArchive.h>

#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace mrpt::img;
// ------------------------------------------------------
//				TestOffscreenRender
// ------------------------------------------------------
int TestOffscreenRender(int argc, char* argv[])
{
	if (argc != 5)
	{
		cerr
			<< "Usage: " << argv[0]
			<< " <FILE.3Dscene> <WIDTH> <HEIGHT> <OUTPUT_IMAGE>\n"
			   " Output image can be in many formats: .png, .jpg, .tif, etc.\n";
		return 1;
	}

	const string sFil = argv[1];
	const int width = atoi(argv[2]);
	const int height = atoi(argv[3]);
	const string sOut = argv[4];

	ASSERT_GT_(width, 0);
	ASSERT_GT_(height, 0);

	COpenGLScene scene;
	CFileGZInputStream f(sFil);
	archiveFrom(f) >> scene;

	CFBORender render(width, height);
	CImage frame(width, height, CH_RGB);

	// here you can put your preferred camera rendering position
	{
		CCamera& camera = render.getCamera(scene);
		camera.setOrthogonal(true);
		camera.setZoomDistance(70);
		camera.setAzimuthDegrees(-90);
		camera.setElevationDegrees(90);
	}

	// render the scene
	render.render_RGB(scene, frame);
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
		return TestOffscreenRender(argc, argv);
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	
}
