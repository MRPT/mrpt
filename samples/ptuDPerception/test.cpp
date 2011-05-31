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

#include <mrpt/hwdrivers/CPtuDPerception.h>

#include <iostream>
#include <stdlib.h>

using namespace std;
using namespace mrpt::hwdrivers;

void showMenu();
void showInfo(CPtuBase *ptu);

int main(){
	
	CPtuBase *ptu = new CPtuDPerception();

	string port;
	int tWait;
	float initial, final, initialRad, finalRad;
	double graPre,radPre;
	char c,axis;

	// Obtains serial port to connect

	cout << "<Pan & Tilt Unit test application>" << endl;
	cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0): ";
	getline(cin,port);
	
	// Ptu initialization

	if(!ptu->init(port))
		cout << endl << "[INFO] Initialitation error";	

	showMenu();
	
	cout << ">> Command: ";
	cin >> c;

	while (c!='q'){

		switch(c){
		
			// Discreet scan
			case 's':	
				// Get parameters for performs a scan
		
				cout << endl << "Enter axis to scan (T for Till or P for Pan): ";
				cin >> axis;	
				cout << endl << "Enter wait time between steps (in milliseconds): ";
				cin >> tWait;	
				cout << endl << "Enter initial position: ";
				cin >> initial;	
				cout << endl << "Enter final position: ";
				cin >> final;
				cout << endl << "Enter precision degrees: ";
				cin >> graPre;	

				// Performs a scan
				initialRad = DEG2RAD(initial);
				finalRad = DEG2RAD(final);
				radPre = DEG2RAD(graPre);

				ptu->scan(axis,tWait,initialRad,finalRad,radPre);

				break;

			// PTU config information
			case 'i':
				
				showInfo(ptu);

				break;

			// Help
			case 'h':

				showMenu();

				break;
			
			// Incorrect command
			default:

				printf("\n Incorrect command \n");

				break;
		}

		
			cout << "\n";
			cout << ">> Command: ";
			cin >> c;
	}



	return 0;
}


/*-------------------------------------------------------------
						  showMenu
-------------------------------------------------------------*/

void showMenu(){
	
	printf("\n======================\n"
		"PTU test application\n"
		"======================\n\n"
		"Commands list:\n"
		"s : performs a discreet scan\n"
		"i : view ptu config information\n"
		"h : this menu\n"
		"q : quit\n\n");

}

/*-------------------------------------------------------------
						 showInfo
-------------------------------------------------------------*/

void showInfo(CPtuBase *ptu){

	double radSec, actRadSec,nRad;
	bool boolean=false;
	char version[150];

	cout << "\nGeneral PTU info:\n\n";

	ptu->echoModeQ(boolean);
	cout << "[INFO] Echo mode: " << boolean << endl;
	ptu->enableLimitsQ(boolean);
	cout << "[INFO] Limits: " << boolean << endl;
	ptu->verboseQ(boolean);
	cout << "[INFO] Verbose: " << boolean << endl;
	ptu->version(version);
	cout << version << endl;

	cout << "\nTilt info:\n\n";

	ptu->acelerationQ('T',radSec);
	cout << "[INFO] Aceleration: " << radSec << endl;
	ptu->baseSpeedQ('T',radSec);
	cout << "[INFO] Base speed: " << radSec << endl;
	ptu->upperSpeedQ('T',radSec);
	cout << "[INFO] Upper speed: " << radSec << endl;
	ptu->lowerSpeedQ('T',radSec);
	cout << "[INFO] Lower speed: " << radSec << endl;
	ptu->speedQ('T',actRadSec);
	cout << "[INFO] Actual speed: " << actRadSec << endl;
	ptu->maxPosQ('T',nRad);
	cout << "[INFO] Max pos: " << nRad << endl;
	ptu->minPosQ('T',nRad);
	cout << "[INFO] Min pos: " << nRad << endl;

	cout << "\nPan info:\n\n";

	ptu->acelerationQ('P',radSec);
	cout << "[INFO] Aceleration: " << radSec << endl;
	ptu->baseSpeedQ('P',radSec);
	cout << "[INFO] Base speed: " << radSec << endl;
	ptu->upperSpeedQ('P',radSec);
	cout << "[INFO] Upper speed: " << radSec << endl;
	ptu->lowerSpeedQ('P',radSec);
	cout << "[INFO] Lower speed: " << radSec << endl;
	ptu->speedQ('P',actRadSec);
	cout << "[INFO] Actual speed: " << actRadSec << endl;
	ptu->maxPosQ('P',nRad);
	cout << "[INFO] Max pos: " << nRad << endl;
	ptu->minPosQ('P',nRad);
	cout << "[INFO] Min pos: " << nRad << endl;

}
