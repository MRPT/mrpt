/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
