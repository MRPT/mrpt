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

#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/hwdrivers/CServoeNeck.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace std;

int main()
{
	try
	{
		CServoeNeck			eNeckBoard;
		std::string			firmVers;

		// Load configuration:
		while ( !mrpt::system::os::kbhit() )
		{
			if (!eNeckBoard.queryFirmwareVersion( firmVers ) )
			{
				cout << "Cannot connect to USB device... Retrying in 1 sec" << endl;
				mrpt::system::sleep(1000);
			}
			else
			{
				cout << "FIRMWARE VERSION: " << firmVers << endl;
				break;
			}
		} // end-while

		cout << "Keys: " << endl;
		cout << "a: turn left" << endl;
		cout << "d: turn right" << endl;
		cout << "r: read angle" << endl;
		cout << "q: run" << endl;
		cout << "c: center" << endl;
		cout << "x: disable servo" << endl;
		cout << "e: enable servo" << endl;
		cout << "f: send a set of angles for testing filtering" << endl;
		cout << "p: send a concrete angle to the servo" << endl;
		cout << "k: send a concrete angle to the servo (fast)" << endl;
		cout << "ESC: end program" << endl << endl;

		//eNeckBoard.enableServo();		// Put the servo at zero position
		eNeckBoard.center();
		int cangle = 0;
		char c;
		do {
			c = os::getch();

			if (c == 0x61)			// 'a' key (turn left)
			{
				cangle += 5;
				mrpt::utils::keep_min(cangle, eNeckBoard.getTruncateFactor()*90 );  // Saturation
				eNeckBoard.setAngle( DEG2RAD(cangle) );
			} // end-if

			if (c == 0x64)			// 'd' key (turn right)
			{
				cangle -= 5;
				mrpt::utils::keep_max(cangle, eNeckBoard.getTruncateFactor()*(-90) );  // Saturation
				eNeckBoard.setAngle( DEG2RAD(cangle) );
			} // end-if

			if (c == 0x72)			// 'r' key (read)
			{
				double nangle;
				if( eNeckBoard.getCurrentAngle( nangle ) )
					std::cout << "Current angle: " << RAD2DEG(nangle) << std::endl;
				else
					std::cout << "Current angle could not be read " << std::endl;
			} // end-if

			if (c == 0x71)			// 'q' key (run)
			{
				eNeckBoard.center();
				mrpt::system::sleep( 200 );
				eNeckBoard.setAngle( DEG2RAD(-90) );
				mrpt::system::sleep( 200 );
				eNeckBoard.setAngle( DEG2RAD(90) );
				mrpt::system::sleep( 200 );

				std::cout << "Performing a complete run ..." << std::endl;
			} // end-if

			if (c == 0x63)			// 'c' key (center)
			{
				eNeckBoard.center();
				mrpt::system::sleep( 200 );
				std::cout << "Centering ..." << std::endl;
			} // end-if

			if (c == 0x78)			// 'x' key (center)
			{
				eNeckBoard.disableServo();
				std::cout << "Servo disabled ..." << std::endl;
			} // end-if

			if (c == 0x65)			// 'x' key (center)
			{
				eNeckBoard.enableServo();
				std::cout << "Servo enabled ..." << std::endl;
			} // end-if

			if (c == 0x66)		// 'f' key (filter)
			{
				double angles[10] = {0, 12, 22, 35, 42, 31, 26, 40, 25, -12 };
				for( unsigned int i = 0; i < 10; i++ )
				{
					eNeckBoard.setAngleWithFilter( DEG2RAD( angles[i] ) );
					mrpt::system::sleep( 200 );
				}
			}

			if (c == 0x70)		// 'p' key (concrete value)
			{
				std::cout << "Insert the angle: ";
				double angle;
				std::cin >> angle;
				eNeckBoard.setAngle( DEG2RAD( angle ) );
			}

			if (c == 0x6B)		// 'k' key (concrete value fast)
			{
				std::cout << "Insert the angle: ";
				double angle;
				std::cin >> angle;
				eNeckBoard.setAngle( DEG2RAD( angle ), 0, true );
			}
			mrpt::system::sleep( 200 );

		} while( c != 27 ); // end-do-while (Esc key)

		eNeckBoard.disableServo();

	} // end-try
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}

	return 0;
}

