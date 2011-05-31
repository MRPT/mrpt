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

#include <mrpt/hwdrivers/CPtuHokuyo.h>

#include <iostream>
#include <stdlib.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace std;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::math;

// Adicional functions

void showMenu();
void showInfo(CPtuHokuyo *ph);


/*-------------------------------------------------------------
						      main
-------------------------------------------------------------*/

int main(){

	CPtuHokuyo ph;

	string portPtu, portHokuyo;
	double initial,final,graPre,speed;
	double initialRad,finalRad,radPre;
	double resolution=0;
	long steps;
	int n_mean, tWait = 0, ptu_type;
	char c,axis,interlaced;
	bool inter;
	CSimplePointsMap		theMap,*mapPtr;

	cout << "PTU & Hokuyo aplication"<< endl;

	// Get comunication serial ports

	cout << endl << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0) for PTU: ";
	getline(cin,portPtu);
	cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0) for Hokuyo: ";
	getline(cin,portHokuyo);
	cout << "Enter the type of PTU (0:DirectPercecption, 1:MICOS): ";
	cin >> ptu_type;

	ph.m_ptu_type = ptu_type;

	// Init Ptu and Hokuyo

	if(!ph.init(portPtu,portHokuyo)){
		mrpt::system::sleep(3000);
		exit(-1);
	}

	showMenu();

	cout << ">> Command: ";
	cin >> c;

	while (c!='q'){
		switch(c){
			// Discreet scan
			case 's':

				// Get parameters for the scan
				
				if (!ptu_type) //
				{
					cout << endl << "Enter axis to scan (T for Tilt or P for Pan): ";
					cin >> axis;
					cout << endl << "Enter wait time between steps (in milliseconds): ";
					cin >> tWait;
					cout << endl << "Enter initial position: ";
					cin >> initial;
					cout << endl << "Enter final position: ";
					cin >> final;
					cout << endl << "Perform double sweep? (y o n): ";
					cin >> interlaced;

					if(interlaced=='y'){
						inter = true;
					}else{
						inter = false;
					}

					cout << endl << "Number of observations for the mean: ";
					cin >> n_mean;

					cout << endl << "Enter precision degrees: ";
					cin >> graPre;

					// Translate degrees to radians

					initialRad = DEG2RAD(initial);
					finalRad = DEG2RAD(final);
					radPre = DEG2RAD(graPre);

					// Calculate real precision

					if((axis=='T')||(axis=='t')){
						resolution = ph.ptu->tiltResolution;
					}else if ((axis=='P')||(axis=='T')){
						resolution = ph.ptu->panResolution;
					}
					steps = ph.ptu->radToPos(axis,radPre);
					cout << endl << "[INFO] Real degrees precision: " << RAD2DEG(resolution*steps) << endl;
					cout << "[INFO] Displacement for second scan: " << resolution*ph.ptu->radToPos('T',graPre/2)<< endl;

					// Performs a complete scan

					ph.scan(axis,tWait,initialRad,finalRad,radPre,n_mean,inter);

					mapPtr = &theMap;

					//ph.limit(theMap);
					//ph.saveMap2File(theMap,"Puntos.pts");
					ph.saveVObs2File("Puntos.rawlog");

					c='q';
				} 
				else if (ptu_type == 1)
				{
					cout << endl << "[ERROR] TU Micos not support this scan mode" << endl;
				}


				break;

			// Continuous scan
			case 'c':

				// Get parameters for the scan

				cout << endl << "Enter axis to scan (T for Tilt or P for Pan): ";
				cin >> axis;
				cout << endl << "Enter PTU speed: ";
				cin >> speed;
				cout << endl << "Enter initial position: ";
				cin >> initial;
				cout << endl << "Enter final position: ";
				cin >> final;

				// Translate degrees to radians

				initialRad = DEG2RAD(initial);
				finalRad = DEG2RAD(final);

				ph.continuousScan(axis, speed , initialRad , finalRad);

				mapPtr = &theMap;

				//ph.limit(theMap);
				//ph.saveMap2File(theMap,"Puntos.pts");
				ph.saveVObs2File("Puntos.rawlog");
				ph.savePitchAndDistances2File();

				c='q';

				break;

			// PTU config information
			case 'i':

				showInfo(&ph);

				cout << "\n";
				cout << ">> Command: ";
				cin >> c;

				break;

			// Help
			case 'h':

				showMenu();
				cout << ">> Command: ";
				cin >> c;

				break;

			// Incorrect command
			default:

				printf("\n Incorrect command \n\n");
				cout << ">> Command: ";
				cin >> c;

				break;
		}

	}

	return 0;

}


/*-------------------------------------------------------------
						  showMenu
-------------------------------------------------------------*/

void showMenu(){

	printf("\n===============================\n"
		"PTU & Hokuyo test application\n"
		"===============================\n\n"
		"Commands list:\n"
		"s : performs a discreet scan\n"
		"c : performs a continuous scan\n"
		"i : view ptu config information\n"
		"h : this menu\n"
		"q : quit\n\n");

}

/*-------------------------------------------------------------
						 showInfo
-------------------------------------------------------------*/

void showInfo(CPtuHokuyo *ph){

	double radSec, actRadSec,nRad;
	bool boolean=false;
	char *version="";

	cout << "\nGeneral PTU info:\n\n";

	ph->ptu->echoModeQ(boolean);
	cout << "[INFO] Echo mode: " << boolean << endl;
	ph->ptu->enableLimitsQ(boolean);
	cout << "[INFO] Limits: " << boolean << endl;
	ph->ptu->verboseQ(boolean);
	cout << "[INFO] Verbose: " << boolean << endl;
	ph->ptu->version(version);
	cout << version << endl;

	cout << "\nTilt info:\n\n";

	ph->ptu->acelerationQ('T',radSec);
	cout << "[INFO] Aceleration: " << radSec << endl;
	ph->ptu->baseSpeedQ('T',radSec);
	cout << "[INFO] Base speed: " << radSec << endl;
	ph->ptu->upperSpeedQ('T',radSec);
	cout << "[INFO] Upper speed: " << radSec << endl;
	ph->ptu->lowerSpeedQ('T',radSec);
	cout << "[INFO] Lower speed: " << radSec << endl;
	ph->ptu->speedQ('T',actRadSec);
	cout << "[INFO] Actual speed: " << actRadSec << endl;
	ph->ptu->maxPosQ('T',nRad);
	cout << "[INFO] Max pos: " << nRad << endl;
	ph->ptu->minPosQ('T',nRad);
	cout << "[INFO] Min pos: " << nRad << endl;

	cout << "\nPan info:\n\n";

	ph->ptu->acelerationQ('P',radSec);
	cout << "[INFO] Aceleration: " << radSec << endl;
	ph->ptu->baseSpeedQ('P',radSec);
	cout << "[INFO] Base speed: " << radSec << endl;
	ph->ptu->upperSpeedQ('P',radSec);
	cout << "[INFO] Upper speed: " << radSec << endl;
	ph->ptu->lowerSpeedQ('P',radSec);
	cout << "[INFO] Lower speed: " << radSec << endl;
	ph->ptu->speedQ('P',actRadSec);
	cout << "[INFO] Actual speed: " << actRadSec << endl;
	ph->ptu->maxPosQ('P',nRad);
	cout << "[INFO] Max pos: " << nRad << endl;
	ph->ptu->minPosQ('P',nRad);
	cout << "[INFO] Min pos: " << nRad << endl;

}
