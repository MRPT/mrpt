/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
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
