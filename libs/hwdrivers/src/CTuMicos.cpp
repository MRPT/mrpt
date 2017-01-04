/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CTuMicos.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/utils/utils_defs.h>
#include <cstring>
#include <cstdio>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;



/*-------------------------------------------------------------
					    rangeMeasure
-------------------------------------------------------------*/

bool CTuMicos::rangeMeasure() {

	char command[50];

	//char command2[50];
	//sprintf(command2,"%u %s ",axis_index,"nreset");

	//if (!transmit(command2)) return false;

	sprintf(command,"%u %s ",axis_index,"nrm");

	if (!transmit(command)) return false;

	return true;

}

/*-------------------------------------------------------------
						moveToAbsPos
-------------------------------------------------------------*/

bool CTuMicos::moveToAbsPos(char axis,double nRad) {

	char command[300];
	sprintf(command,"%f %u %s", RAD2DEG(nRad), axis_index, "nm");

	if (!transmit(command)) return false;

	return true;
}


/*-------------------------------------------------------------
						absPosQ
-------------------------------------------------------------*/

bool CTuMicos::absPosQ(char axis,double &nRad) {

	return radQuerry(axis,'p',nRad);
}


/*-------------------------------------------------------------
						moveToOffPos
-------------------------------------------------------------*/

bool CTuMicos::moveToOffPos(char axis,double nRad) {

	char command[300];
	sprintf(command,"%f %u %s", RAD2DEG(nRad), axis_index, "nr");

	if (!transmit(command)) return false;

	return true;
}


/*-------------------------------------------------------------
						offPosQ
-------------------------------------------------------------*/

bool CTuMicos::offPosQ(char axis,double &nRad) {

	return radQuerry(axis,'p',nRad);
}


/*-------------------------------------------------------------
						maxPosQ
-------------------------------------------------------------*/

bool CTuMicos::maxPosQ(char axis,double &nRad) {

	return radQuerry('u','l',nRad); // Up limit

}


/*-------------------------------------------------------------
						minPosQ
-------------------------------------------------------------*/

bool CTuMicos::minPosQ(char axis,double &nRad) {

	return radQuerry('l','l',nRad); // Low limit
}


/*-------------------------------------------------------------
						speed
-------------------------------------------------------------*/

bool CTuMicos::speed(char axis,double radSec) {

	return radAsign(axis,'v',radSec);
}

/*-------------------------------------------------------------
						speedQ
-------------------------------------------------------------*/

bool CTuMicos::speedQ(char axis,double &radSec) {

	return radQuerry(axis,'v',radSec);
}


/*-------------------------------------------------------------
						aceleration
-------------------------------------------------------------*/

bool CTuMicos::aceleration(char axis,double radSec2){

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}


/*-------------------------------------------------------------
						acelerationQ
-------------------------------------------------------------*/

bool CTuMicos::acelerationQ(char axis,double &radSec2) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}


/*-------------------------------------------------------------
						baseSpeed
-------------------------------------------------------------*/

bool CTuMicos::baseSpeed(char axis,double radSec) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}

/*-------------------------------------------------------------
						baseSpeedQ
-------------------------------------------------------------*/

bool CTuMicos::baseSpeedQ(char axis,double &radSec) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}


/*-------------------------------------------------------------
						upperSpeed
-------------------------------------------------------------*/

bool CTuMicos::upperSpeed(char axis,double radSec) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}

/*-------------------------------------------------------------
						upperSpeedQ
-------------------------------------------------------------*/

bool CTuMicos::upperSpeedQ(char axis,double &radSec) {

	radSec = DEG2RAD(26);

	return true;
}


/*-------------------------------------------------------------
						lowerSpeed
-------------------------------------------------------------*/

bool CTuMicos::lowerSpeed(char axis,double radSec) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}


/*-------------------------------------------------------------
						lowerSpeedQ
-------------------------------------------------------------*/

bool CTuMicos::lowerSpeedQ(char axis,double &radSec) {

	radSec = DEG2RAD(1);

	return true;
}


/*-------------------------------------------------------------
						enableLimitsQ
-------------------------------------------------------------*/

bool CTuMicos::enableLimitsQ(bool &enable) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}


/*-------------------------------------------------------------
						enableLimits
-------------------------------------------------------------*/

bool CTuMicos::enableLimits(bool set) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}


/*-------------------------------------------------------------
					inmediateExecution
-------------------------------------------------------------*/

bool CTuMicos::inmediateExecution(bool set) {

   	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						aWait
-------------------------------------------------------------*/

bool CTuMicos::aWait(void) {

	return true;
}


/*-------------------------------------------------------------
						haltAll
-------------------------------------------------------------*/

bool CTuMicos::haltAll() {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}

/*-------------------------------------------------------------
						halt
-------------------------------------------------------------*/

bool CTuMicos::halt(char axis) {

	char command[50];
	sprintf(command,"%u %s",axis_index,"nabort");

	if (!transmit(command)) return false;

	return true;
}


/*-------------------------------------------------------------
						reset
-------------------------------------------------------------*/

bool CTuMicos::reset(void) {

	char command[50], command2[50];
	sprintf(command,"%u %s ",axis_index,"nreset");

	if (!transmit(command)) return false;

	sprintf(command2,"%u %s ",axis_index,"ncal");

	mrpt::system::sleep(1000);

	if (!transmit(command2)) return false;

	return true;
}

/*-------------------------------------------------------------
						save
-------------------------------------------------------------*/

bool CTuMicos::save(void) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
					restoreDefaults
-------------------------------------------------------------*/

bool CTuMicos::restoreDefaults(void){

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
				restoreFactoryDefaults
-------------------------------------------------------------*/

bool CTuMicos::restoreFactoryDefaults(void){

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						version
-------------------------------------------------------------*/

bool CTuMicos::version(char * sVersion) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						nversion
-------------------------------------------------------------*/

void CTuMicos::nversion(double &nVersion) {

	if(!radQuerry(0,'n',nVersion))
		throw std::runtime_error("INCORRECT VERSION");
}


/*-------------------------------------------------------------
						powerModeQ
-------------------------------------------------------------*/

bool CTuMicos::powerModeQ(bool transit,char &mode){

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						powerMode
-------------------------------------------------------------*/

bool CTuMicos::powerMode(bool transit,char mode){

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						clear
-------------------------------------------------------------*/

bool CTuMicos::clear() {

	char command[300];
	sprintf(command,"%u %s", axis_index, "nclear");

	if(!transmit(command)) return false;

	return true;
}


/*-------------------------------------------------------------
						setLimits
-------------------------------------------------------------*/

bool CTuMicos::setLimits(char axis, double &l, double &u) {

	char command[300]="";
	sprintf(command,"%f %f %u setnlimit", l, u, axis_index);

	if(!transmit(command)) return false;

	return true;
}


/*-------------------------------------------------------------
					   changeMotionDir
-------------------------------------------------------------*/

bool CTuMicos::changeMotionDir() {

	double motionDir;
	unsigned int newMotionDir;
	char command[300]="";

	// Otains actual motion dir
	if (!radQuerry(0,'c',motionDir))
		return false;

	if(!motionDir)
		newMotionDir = 1;
	else
		newMotionDir = 0;

	// Change motion direction
	sprintf(command,"%u %u setmotiondir", newMotionDir, axis_index);

	if(!transmit(command))
		return false;

	return true;
}

/*-------------------------------------------------------------
							init
-------------------------------------------------------------*/

bool CTuMicos::init(const string &port){

	try{


		serPort.open(port);

		cout << endl << "[INFO] Start Tu MICOS comunication config:" << endl;

		cout << "[PTU::OpenSerialPort] Opening serial port...";

		if(serPort.isOpen()) {

			cout << "OK" << endl;

		}else{

			cout << " Error opening serial port";
			return false;

		}

		cout << "[PTU::SetTimeouts] Setting timeouts...";
		serPort.setTimeouts(1000, 1, 1000, 1, 1000);
		cout << "OK" << endl;

		cout << "[PTU::setBaudRate] Setting baud rate...";
		serPort.setConfig(19200);
		cout << "OK" << endl;

		// PTU initial configuration
		cout << "[PTU::setInitialConfiguration] Setting initial configuration...";

		axis_index = 1;
		double version;
		nversion(version);
		if ((!version) || (!clear())) {

			cout << " Error setting initial configuration";
		    serPort.close();
			return false;

		}

		cout << "OK" << endl;

	}
	catch(std::exception &e)
	{
		MRPT_LOG_ERROR_STREAM << "Error initializating: " << e.what();
		return false;
	}
	catch(...)
	{
		MRPT_LOG_ERROR_STREAM << "Error initializating.";
		return false;
	}

	return true;
}


/*-------------------------------------------------------------
						    close
-------------------------------------------------------------*/

void CTuMicos::close(){

	// Check if serPort is open
	if (serPort.isOpen()) {

		serPort.close();
		cout << endl << "[INFO] TuMICOS Serial port closed" << endl;

	}
}


/*-------------------------------------------------------------
						  radError
-------------------------------------------------------------*/

double CTuMicos::radError(char axis,double nRadMoved) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						  transmit
-------------------------------------------------------------*/

bool CTuMicos::transmit(const char * command) {

	char str[300]="";

	// Copy command in str char
	strcpy(str,command);
	strcat(str," ");

	// Wirte in serial port
	size_t written = serPort.Write(str,strlen(str));

	if (!written){
		return false;
	}

	return true;
}


/*-------------------------------------------------------------
						  receive
-------------------------------------------------------------*/

bool CTuMicos::receive(const char * command,char * response) {

	int cnt=0;
	unsigned long nReaden;
	char str[150]; //="";
	//char * tmp="";

    do {
 		   nReaden=serPort.Read(&str[cnt],1);
		   if (nReaden!=0) cnt++;
    } while ( (nReaden!=0) && (str[cnt-1]!='\n') );

 	if (nReaden==0)  return false;

	//cout << str << endl;

	if (str[0])
	{
		strcpy(response,str);
		return true;
	}

    return false;

}


/*-------------------------------------------------------------
						  verboseQ
-------------------------------------------------------------*/

bool CTuMicos::verboseQ(bool &mode) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						  verbose
-------------------------------------------------------------*/


bool CTuMicos::verbose(bool set) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						  echoModeQ
-------------------------------------------------------------*/

bool CTuMicos::echoModeQ(bool &mode) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						  echoMode
-------------------------------------------------------------*/

bool CTuMicos::echoMode(bool mode) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						  resolution
-------------------------------------------------------------*/

bool CTuMicos::resolution(void) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}

/*-------------------------------------------------------------
						 status
-------------------------------------------------------------*/

double CTuMicos::status( double &rad ) {

	return radQuerry(0,'s',rad);

}


/*-------------------------------------------------------------
						 radQuerry
-------------------------------------------------------------*/

bool CTuMicos::radQuerry(char axis,char command,double &rad) {

	char response[150];
	char command2[300];
	bool toRad = true, select = false;

	if (command == 'p') { // Actual position
		sprintf(command2,"%u %s", axis_index, "np");
	} else if (command == 'v') { // Velocity for move
		sprintf(command2,"%u %s", axis_index, "gnv");
	} else if (command == 's') { // Actual status
		sprintf(command2,"%u %s", axis_index, "nst");
		toRad = false;
	} else if (command == 'e') { // Errors
		sprintf(command2,"%u %s", axis_index, "gne");
		toRad = false;
	} else if (command == 'l') { // Limits of the travel
		sprintf(command2,"%u %s", axis_index, "getnlimit");
		select = true;
	} else if (command == 'n') { // Number of version
		sprintf(command2,"%u %s", axis_index, "nversion");
		toRad = false;
	} else if (command == 'c') { // Motion direction
		sprintf(command2,"%u %s", axis_index, "getmotiondir");
		toRad = false;
	}

	if ( ( !transmit(command2) ) || (!receive(NULL,response)) ) return false;

	// If we can convert the result to radians
	if (toRad) {

		// If is necesary to select a part of the reponse
		if (select) {

			char s2[] = " ";
			char *ptr1, *ptr2;
			char *strContext;
			ptr1 = mrpt::system::strtok( response, s2, &strContext );
			ptr2 = mrpt::system::strtok( NULL , s2, &strContext );
			if (axis == 'l')
				rad = (long)atof((const char*)ptr1);
			else
				rad = (long)atof((const char*)ptr2);

		} else {

			// Else converts deegres to radians
			rad = DEG2RAD((double)atof((const char*)response));
		}

	} else {

		// Else converts char to long
		rad = (long)atof((const char*)response);

	}

   	return true;
}


/*-------------------------------------------------------------
						  radAsign
-------------------------------------------------------------*/

bool CTuMicos::radAsign(char axis,char command,double nRad) {


	char command2[300];

	if (command == 'v') {
		sprintf(command2,"%f %u %s", RAD2DEG(nRad), axis_index, "snv");
	}

	return transmit(command2);
}


/*-------------------------------------------------------------
						   scan
-------------------------------------------------------------*/

bool CTuMicos::scan(char axis, int tWait, float initial, float final, double radPre){

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;
}


/*-------------------------------------------------------------
						 radToPos
-------------------------------------------------------------*/

long CTuMicos::radToPos(char axis,double nrad) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
						 posToRad
-------------------------------------------------------------*/

double CTuMicos::posToRad(char axis,long nPos) {

	cout << endl << "[ERROR] Command not defined for this PTunit" << endl;

	return false;

}


/*-------------------------------------------------------------
					    convertToLong
-------------------------------------------------------------*/

long CTuMicos::convertToLong(char *sLong) {

	long a = (long)atof((const char*)sLong);

	return a;
}


/*-------------------------------------------------------------
					   convertToDouble
-------------------------------------------------------------*/

double CTuMicos::convertToDouble(char *sDouble) {

 char * result=strpbrk(sDouble,"-0123456789");
 char * stop;

 return strtod(result,&stop);
}


/*-------------------------------------------------------------
					   checkError
-------------------------------------------------------------*/

int CTuMicos::checkErrors(){

	double code=0;

	radQuerry(0,'e',code);

	if ((int)code == 0) {

		cout << endl << "[No Error]" << endl;

	} else {

		switch ((int)code) {

			case 1:
			case 2:
			case 3:
			case 4:
				cout << endl << "[Error] Internal error" << endl;
				break;
			case 1001:
				cout << endl << "[Error] Wrong parameter type" << endl;
				break;
			case 1002:
				cout << endl << "[Error] Insufficient parameters on the stack" << endl;
				break;
			case 1003:
				cout << endl << "[Error] Value range is exceeded" << endl;
				break;
			case 1004:
				cout << endl << "[Error] Movement range should be exceeded" << endl;
				break;
			case 1008:
				cout << endl << "[Error] Insufficient parameters on the stack" << endl;
				break;
			case 1015:
				cout << endl << "[Error] Parameter out of the movement area" << endl;
				break;
			case 2000:
				cout << endl << "[Error] Unknown command" << endl;
				break;
			default:
				break;
		}
	}

	return code;
}
