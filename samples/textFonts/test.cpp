/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

	img.filledRectangle(0,0,400,300, 0x505050);

	img.selectTextFont("6x13");
	img.textOut(10,10,"Hello World! with font \"6x13\"", 0xFFFFFF);

	img.selectTextFont("6x13B");
	img.textOut(10,30,"Hello World! with font \"6x13B\"", 0xFFFFFF);

	img.selectTextFont("6x13O");
	img.textOut(10,50,"Hello World! with font \"6x13O\"", 0xFFFFFF);

	img.selectTextFont("9x15");
	img.textOut(10,70,"Hello World! with font \"9x15\"", 0xFFFFFF);

	img.selectTextFont("9x15B");
	img.textOut(10,90,"Hello World! with font \"9x15B\"", 0xFFFFFF);

	img.selectTextFont("18x18ja");
	img.textOut(10,110,"MRPTのフォントは易しいです!", 0xFFFFFF);

	img.selectTextFont("10x20");
	img.textOut(10,130,"Hello World! with font \"10x20\"", 0xFFFFFF);


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
