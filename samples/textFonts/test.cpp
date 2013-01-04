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
