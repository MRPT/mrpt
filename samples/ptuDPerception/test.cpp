/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
