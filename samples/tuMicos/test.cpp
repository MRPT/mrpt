/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CTuMicos.h>
#include <mrpt/utils/utils_defs.h> // DEG2RAD

#include <iostream>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;

void showMenu();

int main(){

	CPtuBase *ptu = new CTuMicos();

	string port, c;
	float auxF1, auxF2;
	double auxD1, auxD2, auxD3;

	// Obtains serial port to connect

	cout << "<Pan & Tilt Unit test application>" << endl;
	cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0): ";
	getline(cin,port);

	// Ptu initialization

	if(!ptu->init(port))
		cout << endl << "[INFO] Initialitation error";

	showMenu();

	cout << endl << endl << ">> Command: ";
	cin >> c;

	while (c != "quit") {

		if (c == "ma") {

			cout << endl << "Enter absolute position to move: ";
			cin >> auxF1;
			auxF2 = DEG2RAD(auxF1);
			ptu->moveToAbsPos(0,auxF2);

		} else if (c == "mr") {

			cout << endl << "Enter relative position to move: ";
			cin >> auxF1;
			auxF2 = DEG2RAD(auxF1);
			ptu->moveToOffPos(0,auxF2);

		} else if (c == "apos") {

			ptu->absPosQ(0,auxD1);
			auxD1 = RAD2DEG(auxD1);
			printf("\nActual abs pos: %f", auxD1);

		} else if (c == "rpos") {

			ptu->offPosQ(0,auxD1);
			auxD1 = RAD2DEG(auxD1);
			cout << endl << "Actual off pos: " << auxD1;

		}else if (c == "halt") {

			if (ptu->halt(0))
				cout << endl << "Stoped";

		} else if (c == "maxlimit") {

			ptu->maxPosQ(0,auxD1);
			cout << endl << "Max travel limit: " << auxD1;

		} else if (c == "minlimit") {

			ptu->minPosQ(0,auxD1);
			cout << endl << "Min travel limit: " << auxD1;

		} else if (c == "setlimits") {

			cout << endl << "Enter new travel limits: ";
			cin >> auxD2 >> auxD3;
			ptu->setLimits(0,auxD2,auxD3);

		} else if (c == "speed") {

			ptu->speedQ(0,auxD1);
			auxF1 = RAD2DEG(auxD1);
			cout << endl << "Actual speed: " << auxF1;

		} else if (c == "setspeed") {

			cout << endl << "Enter new Tu speed: ";
			cin >> auxF1;
			auxF2 = DEG2RAD(auxF1);
			ptu->speed(0,auxF2);

		} else if (c=="changedir") {

			if (ptu->changeMotionDir())
				cout << endl << "Changed motion direction";
			else
				cout << endl << "Error in change motion dir";

		} else if (c == "status") {

			ptu->status(auxD1);
			cout << endl << "Status: " << auxD1;

		} else if (c == "nversion") {

			ptu->nversion(auxD1);
			cout << endl << "Firmawre version: " << auxD1;

		} else if (c == "reset") {

			if(ptu->reset())
				cout << endl << "Reset OK";

		} else if (c == "rangeMeasure") {

			if(ptu->rangeMeasure())
				cout << endl << "Search limit forward OK";

		} else if (c == "errors") {

			ptu->checkErrors();

		} else if (c == "help") {

			showMenu();

		} else {
			cout << endl << "Unknow command ";
		}

		cout << endl << endl << ">> Command: ";
		cin >> c;

	}

	return 0;
}


/*-------------------------------------------------------------
						  showMenu
-------------------------------------------------------------*/

void showMenu(){

	printf("\n======================\n"
		"TU test application\n"
		"======================\n\n"
		"Commands list:\n\n"
		"ma       : Performs a absolute movement\n"
		"mr       : Performs a discrete movement\n"
		"apos rpos: Get actual position\n"
		"halt     : Stop movement\n"
		"maxlimit : Get actual max limit of movement\n"
		"minlimit : Get actual min limit of movement\n"
		"setlimits: Set limits of movemet\n"
		"speed    : Get the actual Tu speed\n"
		"setspeed : Set Tu speed\n"
		"changedir: Change motion direction\n"
		"status   : Check if Tu is moving\n"
		"reset    : Reset Tu controler\n"
		"nversion : Firmware version\n"
		"rangeMeasure: Search limit forward"
		"errors   : Check errors\n"
		"help     : This menu\n"
		"quit     : Quit program\n\n");

}
