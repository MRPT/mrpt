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

#include <mrpt/gui.h>
#include <mrpt/utils.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("imageBasics/") );

// ------------------------------------------------------
//				TestImageCap
// ------------------------------------------------------
void TestImageConversion()
{
	// BMP -> JPEG conversion tester:
	// --------------------------------
	CImage		img,img2;
	CTicTac			tictac;
	CTimeLogger     timlog;

	tictac.Tic();
	if (!img.loadFromFile(myDataDir+string("frame_color.jpg")))
	{
		cerr << "Cannot load " << myDataDir+string("frame_color.jpg") << endl;
		return;
	}
	printf("Image loaded in %.03fms\n", 1000*tictac.Tac() );

	if (false)   // A very simple test:
	{
		CDisplayWindow		win1("JPEG file, color");
		win1.setPos(10,10);

		win1.showImage( img );

		cout << "Push a key in the console or in the window to continue...";
		win1.waitForKey();
		cout << "Done" << endl;

		timlog.enter("grayscale1");
		img = img.grayscale();
		timlog.leave("grayscale1");

		CDisplayWindow		win2("JPEG file, gray");
		win2.showImage( img );
		win1.setPos(300,10);

		cout << "Push a key in the console or in the window to continue...";
		win2.waitForKey();
		cout << "Done" << endl;

		mrpt::system::pause();
		return;
	}

	CDisplayWindow		win1("win1"),win2("win2"),win3("win3"),win4("win4");

	CImage			imgSmall( img );
	CImage			imgGray;

	for (int i=0;i<50;i++)
	{
		timlog.enter("grayscale2");
		imgSmall.grayscale(imgGray);
		timlog.leave("grayscale2");
	}

	CImage			imgSmall2( imgGray.scaleHalfSmooth() );
	CImage			imgSmallRGB( img.scaleHalf() ); //Smooth() );

	// Test some draw capabilities:
	// ---------------------------------
	imgSmall.rectangle( 85,35, 170,170,TColor(255,0,0),10);

	imgSmall.line( 550,75, 650,25,TColor(0,0,255));
	imgSmall.line( -10,-20, 20,30,TColor(0,0,255));

	CMatrix 	COV(2,2);
	COV(0,0) = 100;
	COV(1,1) = 50;
	COV(0,1) = COV(1,0) = -30;
	imgSmall.ellipseGaussian( &COV, 600.0f,50.0f, 2, TColor(255,255,0), 4);
	imgGray.ellipseGaussian( &COV, 100.0f,100.0f, 2, TColor(0,0,255), 4);

	imgSmall.drawImage( 400,500,imgGray );

	// Show the windows now:
	// ------------------------------------------------------
	win1.showImage( imgSmall ); win1.setPos(0,0);
	win2.showImage( imgSmall2 ); win2.setPos(810,0);
	win3.showImage( imgGray ); win3.setPos(810,300);
	win4.showImage( imgSmallRGB ); win4.setPos(300,400);



	cout << "Press any key on 'win4' to exit" << endl;
	win4.waitForKey();

	tictac.Tic();
	img2.saveToFile("frame_out.jpg");
	printf("jpeg file saved in %.03fms\n", 1000.0f*tictac.Tac() );

	imgSmall2.saveToFile("frame_out_small.png");

	return;
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestImageConversion();
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
