/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CPtuDPerception.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/os.h>
#include <cstring>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;


/*-------------------------------------------------------------
						moveToAbsPos
-------------------------------------------------------------*/

bool CPtuDPerception::moveToAbsPos(char axis,double nRad) {

	if (!radAsign(axis,'P',nRad)) return false;

	return true;
}


/*-------------------------------------------------------------
						absPosQ
-------------------------------------------------------------*/

bool CPtuDPerception::absPosQ(char axis,double &nRad) {

	return radQuerry(axis,'P',nRad);
}


/*-------------------------------------------------------------
						moveToOffPos
-------------------------------------------------------------*/

bool CPtuDPerception::moveToOffPos(char axis,double nRad) {

	if (!radAsign(axis,'O',nRad)) return false;

	return true;
}


/*-------------------------------------------------------------
						offPosQ
-------------------------------------------------------------*/

bool CPtuDPerception::offPosQ(char axis,double &nRad) {

	return radQuerry(axis,'O',nRad);
}


/*-------------------------------------------------------------
						maxPosQ
-------------------------------------------------------------*/

bool CPtuDPerception::maxPosQ(char axis,double &nRad) {

	return radQuerry(axis,'X',nRad);
}


/*-------------------------------------------------------------
						minPosQ
-------------------------------------------------------------*/

bool CPtuDPerception::minPosQ(char axis,double &nRad) {

	return radQuerry(axis,'N',nRad);
}


/*-------------------------------------------------------------
						speed
-------------------------------------------------------------*/

bool CPtuDPerception::speed(char axis,double radSec) {

	return radAsign(axis,'S',radSec);
}

/*-------------------------------------------------------------
						speedQ
-------------------------------------------------------------*/

bool CPtuDPerception::speedQ(char axis,double &radSec) {

	return radQuerry(axis,'S',radSec);
}


/*-------------------------------------------------------------
						aceleration
-------------------------------------------------------------*/

bool CPtuDPerception::aceleration(char axis,double radSec2){

	return radAsign(axis,'A',radSec2);
}


/*-------------------------------------------------------------
						acelerationQ
-------------------------------------------------------------*/

bool CPtuDPerception::acelerationQ(char axis,double &radSec2) {

	return radQuerry(axis,'A',radSec2);
}


/*-------------------------------------------------------------
						baseSpeed
-------------------------------------------------------------*/

bool CPtuDPerception::baseSpeed(char axis,double radSec) {

	return radAsign(axis,'B',radSec);
}

/*-------------------------------------------------------------
						baseSpeedQ
-------------------------------------------------------------*/

bool CPtuDPerception::baseSpeedQ(char axis,double &radSec) {

	return radQuerry(axis,'B',radSec);
}


/*-------------------------------------------------------------
						upperSpeed
-------------------------------------------------------------*/

bool CPtuDPerception::upperSpeed(char axis,double radSec) {

	return radAsign(axis,'U',radSec);
}

/*-------------------------------------------------------------
						upperSpeedQ
-------------------------------------------------------------*/

bool CPtuDPerception::upperSpeedQ(char axis,double &radSec) {

	return radQuerry(axis,'U',radSec);
}


/*-------------------------------------------------------------
						lowerSpeed
-------------------------------------------------------------*/

bool CPtuDPerception::lowerSpeed(char axis,double radSec) {

	return radAsign(axis,'L',radSec);
}


/*-------------------------------------------------------------
						lowerSpeedQ
-------------------------------------------------------------*/

bool CPtuDPerception::lowerSpeedQ(char axis,double &radSec) {

	return radQuerry(axis,'L',radSec);
}


/*-------------------------------------------------------------
						enableLimitsQ
-------------------------------------------------------------*/

bool CPtuDPerception::enableLimitsQ(bool &enable) {

	char response[150];

	if (!transmit("L") || !receive("L",response)) return false;

    if (strstr( upperCase(response).c_str(),"ENABLE")!=NULL) enable=true;
	else enable=false;

	return true;
}


/*-------------------------------------------------------------
						enableLimits
-------------------------------------------------------------*/

bool CPtuDPerception::enableLimits(bool set) {

	if (set) return ( transmit("LE") && receive("LE",NULL) );
	else return (transmit("LD") && receive("LD",NULL) );
}


/*-------------------------------------------------------------
					inmediateExecution
-------------------------------------------------------------*/

bool CPtuDPerception::inmediateExecution(bool set) {

   if (set) return ( transmit("I") && receive("I",NULL) );
   else return ( transmit("S") && receive("S",NULL) );

}


/*-------------------------------------------------------------
						aWait
-------------------------------------------------------------*/

bool CPtuDPerception::aWait(void) {

	return ( transmit("A") && receive("A",NULL) );
}


/*-------------------------------------------------------------
						haltAll
-------------------------------------------------------------*/

bool CPtuDPerception::haltAll() {

	return ( transmit("H") && receive("H",NULL) );
}

/*-------------------------------------------------------------
						halt
-------------------------------------------------------------*/

bool CPtuDPerception::halt(char axis) {

	char sTrans[3];
    sTrans[0]='H';sTrans[1]=axis;sTrans[2]='\0';

	return ( transmit(sTrans) && receive(sTrans,NULL) );
}


/*-------------------------------------------------------------
						reset
-------------------------------------------------------------*/

bool CPtuDPerception::reset(void) {

    if (!transmit("R")) return false;
	receive("R",NULL);

	return panTiltHitError();
}

/*-------------------------------------------------------------
						save
-------------------------------------------------------------*/

bool CPtuDPerception::save(void) {

	return ( transmit("DS") && receive("DS",NULL) );
}


/*-------------------------------------------------------------
					restoreDefaults
-------------------------------------------------------------*/

bool CPtuDPerception::restoreDefaults(void){

	return ( transmit("DR") && receive("DR",NULL) );
}


/*-------------------------------------------------------------
				restoreFactoryDefaults
-------------------------------------------------------------*/

bool CPtuDPerception::restoreFactoryDefaults(void){

	return ( transmit("DF") && receive("DF",NULL) );
}


/*-------------------------------------------------------------
						version
-------------------------------------------------------------*/

bool CPtuDPerception::version(char * sVersion) {

	return ( transmit("V") && receive("V",sVersion) );

}


/*-------------------------------------------------------------
						powerModeQ
-------------------------------------------------------------*/

bool CPtuDPerception::powerModeQ(bool transit,char &mode){

	const char * response="";

	if (transit)
	{
      if (!transmit("PM")) return false;
	else
	  if (!transmit("PH"))  return false;
	}

	if (strstr(upperCase(response).c_str(),"REGULAR")!=NULL)
		mode=Regular;
	else if (strstr(upperCase(response).c_str(),"HIGH")!=NULL)
        mode=High;
	else if (strstr(upperCase(response).c_str(),"LOW")!=NULL)
        mode=Low;
	else // OFF
        mode=Off;

	return true;
}


/*-------------------------------------------------------------
						powerMode
-------------------------------------------------------------*/

bool CPtuDPerception::powerMode(bool transit,char mode){

	char sTrans[4]; //="";
    sTrans[0]='P';
	sTrans[1]= transit ? 'M':'H';
	sTrans[2]=mode;
	sTrans[3]='\0';

	return ( transmit(sTrans) && receive(sTrans,NULL) );
}


/*-------------------------------------------------------------
							init
-------------------------------------------------------------*/

bool CPtuDPerception::init(const string &port){

	try{

		serPort.open(port);

		cout << endl << "[INFO] Start PTU comunication config:" << endl;

		cout << "[PTU::OpenSerialPort] Opening serial port...";

		if(serPort.isOpen()) {

		}else{
			cout << " Error opening serial port";
			return 0;
		}

		cout << "OK" << endl;

		cout << "[PTU::SetTimeouts] Setting timeouts...";
		serPort.setTimeouts(1000,1,1000, 1, 1000);
		cout << "OK" << endl;

		cout << "[PTU::setBaudRate] Setting baud rate...";
		serPort.setConfig(9600);
		cout << "OK" << endl;

		// PTU initial configuration
		cout << "[PTU::setInitialConfiguration] Setting initial configuration...";
		if ( (!verbose(true)) ||	// Original: false	Actual: true
			 (!resolution()) ||
			 (!echoMode(true)) ||
			 (!inmediateExecution(true))
			 ) {
			cout << " Error setting initial configuration";
		    serPort.close();
			return false;
		}

		cout << "OK" << endl << endl << "[INFO] Pan Resolution: " << panResolution << " radians | " << RAD2DEG(panResolution) << "degrees";
		cout << endl << "[INFO] TitlResolution: " << tiltResolution << " radians | " << RAD2DEG(tiltResolution) << "degrees" << endl << endl;

	}
		catch(exception &e)
	{
		cerr << e.what() << endl;
		return 0;
	}

	return true;

}


/*-------------------------------------------------------------
						    close
-------------------------------------------------------------*/

void CPtuDPerception::close(){

	serPort.close();
}


/*-------------------------------------------------------------
						  radError
-------------------------------------------------------------*/

double CPtuDPerception::radError(char axis,double nRadMoved) {

	double div;

	if (axis==Pan)
		div=nRadMoved-long(nRadMoved/panResolution)*panResolution;
	else
		div=nRadMoved-long(nRadMoved/tiltResolution)*tiltResolution;

    return  div;
}


/*-------------------------------------------------------------
						  transmit
-------------------------------------------------------------*/

bool CPtuDPerception::transmit(const char * command) {

	char str[20]="";

	strcpy(str,command);
	strcat(str," ");

	size_t written = serPort.Write(str,strlen(str));

	if (!written){
		return false;
	}

	return true;
}


/*-------------------------------------------------------------
						  receive
-------------------------------------------------------------*/

bool CPtuDPerception::receive(const char * command,char * response) {

	int cnt=0;
	unsigned long nReaden;
	char str[150]="";
	char * tmp;

    do {
		   nReaden=serPort.Read(&str[cnt],1);
		   if (nReaden!=0) cnt++;
    } while ( (nReaden!=0) && (((tmp=strstr(str,command))==NULL) ||
		       (str[cnt-1]!='\n')) );

 	if (nReaden==0) { nError=nError*TimeoutError; return false; }


	if (response!=NULL) {
		//*response=new char[150];
	    strcpy(response,tmp);
	}

	//cout << str << endl;

    if (strstr(tmp,"!")==NULL) { nError=nError*NoError; return true; }

	if ((strstr(tmp,"!P")!=NULL) && (strstr(tmp,"!T")!=NULL )) nError=nError*PanTiltHitError;
	else if (strstr(tmp,"!T")!=NULL ) nError=nError*TiltHitError;
    else if (strstr(tmp,"!P")!=NULL) nError=nError*PanHitError;
	else if (strstr(tmp,"! Maximum")!=NULL) nError=nError*MaxLimitError;
   	else if (strstr(tmp,"! Minimum")!=NULL) nError=nError*MinLimitError;
   	else if (strstr(tmp,"! Value")!=NULL) nError=nError*OutOfRange;
    else if (strstr(tmp,"! Illegal")!=NULL) nError=nError*IllegalCommandError;
	else nError=nError*UnExpectedError;

    return false;

}


/*-------------------------------------------------------------
						  verboseQ
-------------------------------------------------------------*/

bool CPtuDPerception::verboseQ(bool &mode) {

	char response[150];

	if (!transmit("F") || !receive("F",response)) return false;

    if (strstr(response,"VERBOSE")!=NULL) mode=true;
	else mode=false;

	return true;
}


/*-------------------------------------------------------------
						  verbose
-------------------------------------------------------------*/


bool CPtuDPerception::verbose(bool set) {

	if (set) return (transmit("FV") && (receive("FV",NULL)) );
	else return (transmit("FT") && (receive("FT",NULL)) );
}


/*-------------------------------------------------------------
						  echoModeQ
-------------------------------------------------------------*/

bool CPtuDPerception::echoModeQ(bool &mode) {

	char response[150];

	if (!transmit("E") || !receive("E",response)) return false;

    if (strstr(upperCase(response).c_str(),"ENABLE")!=NULL) mode=true;
	else mode=false;

	return true;
}


/*-------------------------------------------------------------
						  echoMode
-------------------------------------------------------------*/

bool CPtuDPerception::echoMode(bool mode) {

	if (mode) return (transmit("EE") && receive("EE",NULL) );
	else return (transmit("ED") && receive("ED",NULL) );
}


/*-------------------------------------------------------------
						  resolution
-------------------------------------------------------------*/

bool CPtuDPerception::resolution(void) {

	char response[150];

	if ( (!transmit("PR")) || (!receive("PR",response)) ) return false;
    panResolution=DEG2RAD(convertToDouble(response) / 3600);

	if ( (!transmit("TR")) || (!receive("TR",response)) ) return false;
    tiltResolution=DEG2RAD(convertToDouble(response) / 3600);

    return true;
}


/*-------------------------------------------------------------
						 radQuerry
-------------------------------------------------------------*/

bool CPtuDPerception::radQuerry(char axis,char command,double &rad) {

	char response[150];
    char sTrans[3];

	sTrans[0]=axis;
	sTrans[1]=command;
	sTrans[2]='\0';


	if ( ( !transmit(sTrans) ) || (!receive(sTrans,response)) ) return false;

    rad=posToRad(axis,convertToLong(response));

   	return true;
}


/*-------------------------------------------------------------
						  radAsign
-------------------------------------------------------------*/

bool CPtuDPerception::radAsign(char axis,char command,double nRad) {

	char sPos[20];
    char sTrans[22];

	char response[150];

	os::sprintf(sPos,sizeof(sPos), "%li", radToPos(axis,nRad));

	sTrans[0]=axis;
	sTrans[1]=command;
	strcpy(&sTrans[2],sPos);

	return (transmit(sTrans) && receive(sTrans,response));
}


/*-------------------------------------------------------------
						   scan
-------------------------------------------------------------*/

bool CPtuDPerception::scan(char axis, int tWait, float initial, float final, double radPre){

	// Check initial and final positions
	if(initial<final){
		float aux = initial;
		initial = final;
		final = aux;
	}

	// Go to initial position
	moveToAbsPos(axis,initial);
	aWait();

	mrpt::system::sleep(500);

	double j=0;
	offPosQ(axis,j);

	long steps = radToPos(axis,radPre);
	long totalSteps;

	// Obtain total number of steps
	int initialPos = radToPos(axis,initial);
	int finalPos = radToPos(axis,final);

	totalSteps = abs(initialPos-finalPos);

	// Performs first sweep
	for(int i=0;i<totalSteps/steps;i++)
	{
		if(initial>final){
			moveToOffPos(axis,-radPre);
		}else{
			moveToOffPos(axis,radPre);
		}
		offPosQ(axis,j);
		mrpt::system::sleep(tWait);
	}

	// Adjust steps for second scan
	moveToOffPos(axis,radPre/2);
	aWait();

	// Performs seecond scan
	for(int i=0;i<(totalSteps/steps)-1;i++)
	{
		if(initial>final){
			moveToOffPos(axis,radPre);
		}else{
			moveToOffPos(axis,-radPre);
		}
		offPosQ(axis,j);
		mrpt::system::sleep(tWait);
	}

	offPosQ(axis,j);

	// Return to initial position
	moveToAbsPos(axis,0);

	return true;
}


/*-------------------------------------------------------------
						 radToPos
-------------------------------------------------------------*/

long CPtuDPerception::radToPos(char axis,double nrad) {

	if (axis==Pan)  return (long) (nrad / panResolution);
	else  return (long) (nrad / tiltResolution);
}


/*-------------------------------------------------------------
						 posToRad
-------------------------------------------------------------*/

double CPtuDPerception::posToRad(char axis,long nPos) {

	if (axis==Pan) return (double) nPos * panResolution;
	else return (double) nPos * tiltResolution;
}


/*-------------------------------------------------------------
					    convertToLong
-------------------------------------------------------------*/

long CPtuDPerception::convertToLong(char *sLong) {

 char * result=strpbrk(sLong,"-0123456789");
 char * stop;

 return strtol(result,&stop,10);
}


/*-------------------------------------------------------------
					   convertToDouble
-------------------------------------------------------------*/

double CPtuDPerception::convertToDouble(char *sDouble) {

 char * result=strpbrk(sDouble,"-0123456789");
 char * stop;

 return strtod(result,&stop);
}


/*-------------------------------------------------------------
					   checkError
-------------------------------------------------------------*/

int CPtuDPerception::checkErrors(){

	int code=0;

	//Check for errors
	if(noError()){
		code = 0;
	}else{
		if(comError()){
			code = 1;
		}else if(timeoutError()){
			code = 2;
		}else if(initError()){
			code = 3;
		}else if(panTiltHitError()){
			code = 4;
		}else if(panHitError()){
			code = 5;
		}else if(tiltHitError()){
			code = 6;
		}else if(maxLimitError()){
			code = 7;
		}else if(minLimitError()){
			code = 8;
		}else if(outOfRange()){
			code = 9;
		}else if(illegalCommandError()){
			code = 10;
		}else if(unExpectedError()){
			code = 11;
		}
	}

	return code;
}

/*-------------------------------------------------------------
					   nVersion
-------------------------------------------------------------*/

void CPtuDPerception::nversion(double &nVersion) {
	cout << "[ERROR] Function not defined for this PTU model";
	nVersion = 0;
}


/*-------------------------------------------------------------
					   setLimits
-------------------------------------------------------------*/

bool CPtuDPerception::setLimits(char axis, double &l, double &u) {
	MRPT_UNUSED_PARAM(axis); MRPT_UNUSED_PARAM(l); MRPT_UNUSED_PARAM(u);
	cout << "[ERROR] Function not defined for this PTU model";
	return false;
}


/*-------------------------------------------------------------
					   changeMotionDir
-------------------------------------------------------------*/

bool CPtuDPerception::changeMotionDir() {
	cout << "[ERROR] Function not defined for this PTU model";
	return false;
}


/*-------------------------------------------------------------
					    rangeMeasure
-------------------------------------------------------------*/

bool CPtuDPerception::rangeMeasure() {
	cout << "[ERROR] Function not defined for this PTU model";
	return false;
}
