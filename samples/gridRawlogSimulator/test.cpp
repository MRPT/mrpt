/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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


#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/base.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::random;
using namespace std;


int main(int argc, char ** argv)
{
    try
    {
		if (argc!=3)
		{
			cerr << "Usage: " << argv[0] << " <GRIDMAP.png> <PIXEL SIZE (meters)>" << endl;
			return -1;
		}

		COccupancyGridMap2D 	gridmap;

		gridmap.loadFromBitmapFile( argv[1], atof(argv[2]) );

		randomGenerator.randomize();

		CDisplayWindowPlots		win("Gridmap 2D simulator");

		CImage	bmpImg;
		gridmap.getAsImage( bmpImg );

		win.image(bmpImg, gridmap.getXMin(), gridmap.getYMin(), gridmap.getSizeX()*gridmap.getResolution(),gridmap.getSizeY()*gridmap.getResolution(), "grid" );
		win.axis_equal();

		CRobotSimulator		robotSim;

		std::string 	outFile("out.rawlog");
		std::string 	outDir("OUT");

		// Create out dir:
		mrpt::system::createDirectory(outDir);

		CFileOutputStream     fil(format("%s/%s",outDir.c_str(),outFile.c_str()) );


		CTicTac tictac;
		tictac.Tic();

		CPose2D   odoPose, realPose;
		double t0 = tictac.Tac();

		for (;;)
		{
			// Real-time simulation:
			double t1 = tictac.Tac();
			double At = t1 - t0;
			t0 = t1;
			robotSim.simulateInterval(At);

			robotSim.getOdometry(odoPose);
			robotSim.getRealPose(realPose);

			cout << "[sim] robot: " << realPose << endl;

			mrpt::system::sleep(20);

			// Process keys:
			if (os::kbhit())
			{
				char c= os::getch();
				printf("C:%i\n",c);
				if (c==27 || c=='q' || c=='Q') break;
			}
		}

    }
    catch(std::exception &e)
    {
        std::cout << e.what();
    }
    catch(...)
    {
        std::cout << "Untyped exception!";
    }
}
