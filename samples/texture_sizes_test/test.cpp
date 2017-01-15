/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CImage.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/system/threads.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/CText.h>

#include <mrpt/examples_config.h>
const std::string   myTestFile( MRPT_EXAMPLES_BASE_DIRECTORY + std::string("imageBasics/frame_color.jpg") );

using namespace std;
using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::utils;


// ------------------------------------------------------
//				TextureSizes_test
// ------------------------------------------------------
void TextureSizes_test()
{
	// Prepare a few test images: color & BW, random size and 2^N size.
	// -------------------------------------------------------------------
	CImage imgCol_N,  imgBW_N;
	CImage imgCol_2N, imgBW_2N;

	if (!imgCol_N.loadFromFile(myTestFile))
	{
		cerr << "Cannot load " << myTestFile << endl;
		return;
	}

	imgCol_N.scaleImage(imgCol_2N,512,512);

	imgCol_N.grayscale(imgBW_N);
	imgCol_2N.grayscale(imgBW_2N);

	// Masks:
	const int W = imgCol_N.getWidth();
	const int H = imgCol_N.getHeight();

	CImage transpMask_N(W,H, CH_GRAY );
	for (int y=0;y<H;y++)
		for (int x=0;x<W;x++)
			*transpMask_N(x,y) = (((x+y)>>5)&1) ? 240 : 10;

	CImage transpMask_2N;
	transpMask_N.scaleImage(transpMask_2N,512,512);


	cout << "Loaded image size: " << imgCol_N.getWidth() << "x" << imgCol_N.getHeight() << endl;
	cout << "2^N image size   : " << imgCol_2N.getWidth() << "x" << imgCol_2N.getHeight() << endl;


	CDisplayWindow3D	win("Test of MRPT's OpenGL textures",640,480);

	COpenGLScenePtr &theScene = win.get3DSceneAndLock();

	double off_x = 0;
	const double off_y_label = 4;
	const double STEP_X = 15;

	if (1)
	{
		opengl::CTexturedPlanePtr obj = opengl::CTexturedPlane::Create(-3,3, -3,3);
		obj->assignImage(imgCol_N);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("Color texture, random size, w/o transp");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	if (1)
	{
		opengl::CTexturedPlanePtr obj = opengl::CTexturedPlane::Create(-3,3, -3,3);
		obj->assignImage(imgCol_N, transpMask_N);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("Color texture, random size, with transp");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	if (1)
	{
		opengl::CTexturedPlanePtr obj = opengl::CTexturedPlane::Create(-3,3, -3,3);
		obj->assignImage(imgBW_N);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("B/W texture, random size, w/o transp");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	if (1)
	{
		opengl::CTexturedPlanePtr obj = opengl::CTexturedPlane::Create(-3,3, -3,3);
		obj->assignImage(imgBW_N, transpMask_N);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("B/W texture, random size, with transp");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	if (1)
	{
		opengl::CTexturedPlanePtr obj = opengl::CTexturedPlane::Create(-3,3, -3,3);
		obj->assignImage(imgCol_2N);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("Color texture, 2^N size, w/o transp");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	if (1)
	{
		opengl::CTexturedPlanePtr obj = opengl::CTexturedPlane::Create(-3,3, -3,3);
		obj->assignImage(imgCol_2N, transpMask_2N);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("Color texture, 2^N size, with transp");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	if (1)
	{
		opengl::CTexturedPlanePtr obj = opengl::CTexturedPlane::Create(-3,3, -3,3);
		obj->assignImage(imgBW_2N);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("B/W texture, 2^N size, w/o transp");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;

	if (1)
	{
		opengl::CTexturedPlanePtr obj = opengl::CTexturedPlane::Create(-3,3, -3,3);
		obj->assignImage(imgBW_2N, transpMask_2N);
		obj->setLocation(off_x,0,0);
		theScene->insert( obj );

		opengl::CTextPtr gl_txt = opengl::CText::Create("B/W texture, 2^N size, with transp");
		gl_txt->setLocation(off_x,off_y_label,0);
		theScene->insert(gl_txt);
	}
	off_x+=STEP_X;



	win.setCameraZoom(150);
	win.setCameraAzimuthDeg(90);

	// IMPORTANT!!! IF NOT UNLOCKED, THE WINDOW WILL NOT BE UPDATED!
	win.unlockAccess3DScene();
	win.repaint();

	cout << "Close the window to end.\n";
	while (win.isOpen())
	{
		win.addTextMessage(5,5, format("%.02fFPS", win.getRenderingFPS()));
		mrpt::system::sleep(2);
		win.repaint();
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TextureSizes_test();

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
