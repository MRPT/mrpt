/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers/CServoeNeck.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace std;

#define SRV1 0
#define SRV2 1

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

		eNeckBoard.enableServo();		// Enable the servos
		eNeckBoard.center(SRV1);		// Center both servos
		eNeckBoard.center(SRV2);

		cout << "Use this keys: " << endl;
		cout << "-------------------------" << endl;

		cout << "b: enable servos" << endl;
		cout << "n: disable servo" << endl;
		cout << "-------------------------" << endl;

		cout << "a: turn left [1]" << endl;
		cout << "s: center [1]" << endl;
		cout << "d: turn right [1]" << endl;
		cout << "f: do a complete run [1]" << endl;

		cout << "j: turn left [2]" << endl;
		cout << "k: center [2]" << endl;
		cout << "l: turn right [2]" << endl;
		cout << "h: do a complete run [2]" << endl;

		cout << "-------------------------" << endl;

		cout << "q: read angle [1]" << endl;
		cout << "w: send a set of angles for testing filtering [1]" << endl;
		cout << "e: send a concrete angle to the servo [1]" << endl;
		cout << "r: send a concrete angle to the servo (fast) [1]" << endl;

		cout << "y: read angle [2]" << endl;
		cout << "u: send a set of angles for testing filtering [2]" << endl;
		cout << "i: send a concrete angle to the servo [2]" << endl;
		cout << "o: send a concrete angle to the servo (fast) [2]" << endl;

		cout << "-------------------------" << endl;
		cout << "ESC: end program" << endl << endl;

		int cangle = 0;
		double nangle, angle, angles[10];
		int srv;
		char c;
		do {
			c = os::getch();

			switch(c)
			{
			case 'a':
			case 'j': // turn left
				srv = c == 'a' ? SRV1 : SRV2;
				cangle += 5;
				mrpt::utils::keep_min(cangle, eNeckBoard.getTruncateFactor()*90 );		// Saturation
				eNeckBoard.setAngle( DEG2RAD(cangle), srv );
				std::cout << "Turning servo " << srv+1 << " 5deg to the left" << std::endl;
				break;

			case 'd':
			case 'l': // turn right
				srv = c == 'd' ? SRV1 : SRV2;
				cangle -= 5;
				mrpt::utils::keep_max(cangle, eNeckBoard.getTruncateFactor()*(-90) );	// Saturation
				eNeckBoard.setAngle( DEG2RAD(cangle), srv );
				std::cout << "Turning servo " << srv+1 << " 5deg to the right" << std::endl;
				break;

			case 's':
			case 'k': // center
				srv = c == 's' ? SRV1 : SRV2;
				eNeckBoard.center(srv);
				mrpt::system::sleep( 200 );
				std::cout << "Centering servo " << srv+1 << std::endl;
				break;

			case 'f':
			case 'h': // complete run
				srv = c == 'f' ? SRV1 : SRV2;
				eNeckBoard.center( srv );
				mrpt::system::sleep( 200 );
				eNeckBoard.setAngle( DEG2RAD(-90), srv );
				mrpt::system::sleep( 200 );
				eNeckBoard.setAngle( DEG2RAD(90), srv );
				mrpt::system::sleep( 200 );
				std::cout << "Performing a complete run in servo " << srv+1 << std::endl;
				break;

			case 'q':
			case 'y': // read angle
				srv = c == 'q' ? SRV1 : SRV2;
				if( eNeckBoard.getCurrentAngle( nangle ), srv )
					std::cout << "Current angle of servo " << srv+1 << " is " << RAD2DEG(nangle) << "deg" << std::endl;
				else
					std::cout << "Current angle of servo " << srv+1 << " could not be read" << std::endl;
				break;

			case 'w':
			case 'u': // send a set of angles
				srv = c == 'w' ? SRV1 : SRV2;
				angles[0] = 0; angles[1] = 12; angles[2] = 22; angles[3] = 35; angles[4] = 42;
				angles[5] = 31;angles[6] = 26; angles[7] = 40; angles[8] = 25; angles[9] = -12;
				for( unsigned int i = 0; i < 10; i++ )
				{
					eNeckBoard.setAngleWithFilter( DEG2RAD( angles[i] ), srv );
					mrpt::system::sleep( 200 );
				}
				std::cout << "Sending a set of angles to servo " << srv+1 << std::endl;
				break;

			case 'e':
			case 'i': // send a concrete angle
				srv = c == 'e' ? SRV1 : SRV2;
				std::cout << "Insert the angle: ";
				std::cin >> angle;
				eNeckBoard.setAngle( DEG2RAD( angle ), srv );
				break;

			case 'r':
			case 'o': // send a concrete angle (fast)
				srv = c == 'r' ? SRV1 : SRV2;
				std::cout << "Insert the angle: ";
				std::cin >> angle;
				eNeckBoard.setAngle( DEG2RAD( angle ), srv, true );
				break;

			case 'b':
				if( eNeckBoard.enableServo() )
					std::cout << "Servos enabled ..." << std::endl;
				else
					std::cout << "Servos could not be enabled ..." << std::endl;
				break;

			case 'n':
				eNeckBoard.disableServo();
				std::cout << "Servos disabled ..." << std::endl;
				break;

			default: continue;
			}
			mrpt::system::sleep( 200 );
		} while( c != 27 ); // end-do-while (Esc key)
		eNeckBoard.disableServo();	// assert that the servo is disbled at the end
	} // end-try
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}

	return 0;
}

