/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace std;

// ------------------------------------------------------
//				TestFonts
// ------------------------------------------------------
void TestFonts()
{
	CImage		img(400,300);

	img.filledRectangle(0,0,400,300, TColor(0x50,0x50,0x50) );

	int y = 10;
	img.selectTextFont("5x7");
	img.textOut(10,y,"Hello World! with font \"5x7\"", TColor::white); y+=20;

	img.selectTextFont("6x13");
	img.textOut(10,y,"Hello World! with font \"6x13\"", TColor::white); y+=20;

	img.selectTextFont("6x13B");
	img.textOut(10,y,"Hello World! with font \"6x13B\"", TColor::white); y+=20;

	img.selectTextFont("6x13O");
	img.textOut(10,y,"Hello World! with font \"6x13O\"", TColor::white); y+=20;

	img.selectTextFont("9x15");
	img.textOut(10,y,"Hello World! with font \"9x15\"", TColor::white); y+=20;

	img.selectTextFont("9x15B");
	img.textOut(10,y,"Hello World! with font \"9x15B\"", TColor::white); y+=20;

	img.selectTextFont("18x18ja");
	img.textOut(10,y,"MRPTのフォントは易しいです!", TColor::white); y+=20;

	img.selectTextFont("10x20");
	img.textOut(10,y,"Hello World! with font \"10x20\"", TColor::white); y+=20;


    CDisplayWindow		win1("MRPT - Demo of text fonts render");
    win1.setPos(10,10);
    win1.showImage( img );

    cout << "Push a key in the console or in the window to continue...";
    win1.waitForKey();
    cout << "Done" << endl;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestFonts();
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
