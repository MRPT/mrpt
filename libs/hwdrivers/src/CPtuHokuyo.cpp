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

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/hwdrivers/CPtuHokuyo.h>
#include <mrpt/hwdrivers/CTuMicos.h>
#include <vector>
#include <mrpt/gui.h>

using namespace mrpt;

using namespace mrpt::gui;
using namespace mrpt::opengl;

using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CPtuHokuyo,mrpt::hwdrivers)


/*-------------------------------------------------------------
						 Construcutor
-------------------------------------------------------------*/

CPtuHokuyo::CPtuHokuyo() :
	m_ptu_port("COM1"),
	m_axis('T'),
	m_velocity(0.1),
	m_initial(30),
	m_final(-40),
	m_hokuyo_frec(0.25),
	ptu(0),
	m_ptu_type(0)
{
}

/*-------------------------------------------------------------
						  Destructor
-------------------------------------------------------------*/

CPtuHokuyo::~CPtuHokuyo(){

	if(ptu)
		delete ptu;

	vObs.erase( vObs.begin(), vObs.end() );
	vObs.clear();
}


/*-------------------------------------------------------------
						  init
-------------------------------------------------------------*/

bool CPtuHokuyo::init(const string &portPtu,const string &portHokuyo){

	cout << endl << "[INFO] Using serial port for PTU: " << portPtu << endl;
	cout << "[INFO] Using serial port for Hokuyo: " << portHokuyo << endl;

	if(!m_ptu_type) {
		high = 0.095;
	} else if (m_ptu_type == 1) {
		high = 0.112;
	}

	if (!m_ptu_type)
		ptu = new CPtuDPerception();
	else if (m_ptu_type == 1)
		ptu = new CTuMicos();

	// Ptu initialization

	if(ptu->init(portPtu)){
		printf("\n[TEST] Initialization PTU OK!\n");
	}else{
		printf("[TEST] Initialization PTU failed!\n");
		return false;
	}

	// Hokuyo initialization
	laser.setSerialPort(portHokuyo);
	laser.setVerbose(false); // Disable warning message upon truncated read frame, due to us doing a "purge"...

	printf("\n[TEST] Turning laser ON...\n");
	if (laser.turnOn())
		printf("[TEST] Initialization Hokuyo OK!\n");
	else
	{
		printf("[TEST] Initialization Hokuyo failed!\n");
		return false;
	}

	return true;

}


/*-------------------------------------------------------------
						  scan
-------------------------------------------------------------*/

bool CPtuHokuyo::scan(char &axis, const int &tWait, double &initial, double &final, const double &radPre, const int &n_mean, const bool &interlaced){

	if(initial<final){
		float aux;
		aux=initial;
		initial = final;
		final = aux;
	}

	// Move PTU to initial position
	ptu->moveToAbsPos(axis,initial);
	ptu->aWait();

	// Obtain total number of steps and movements
	long steps = ptu->radToPos(axis,radPre);

	int initialPos = ptu->radToPos(axis,initial);
	int finalPos = ptu->radToPos(axis,final);

	long totalSteps = abs(initialPos-finalPos);

	int movements = totalSteps/steps;

	// Performs first scan
	if(!singleScan(axis,tWait,movements,-radPre,n_mean))
		return false;

	// Check if we have to perform a second scan
	if(interlaced){

		// Adjust position for perform the second scan
		ptu->moveToAbsPos(axis,initial-radPre/2);
		ptu->aWait();
		mrpt::system::sleep(1500);

		movements=(totalSteps/steps)-1;

		// Performs second scan
		if(!singleScan(axis,tWait,movements,-radPre,n_mean))
			return false;
	}

	// Return to initial position
	ptu->moveToAbsPos(axis,0);

	return true;
}


/*-------------------------------------------------------------
						obtainObs
-------------------------------------------------------------*/

bool CPtuHokuyo::obtainObs(CObservation2DRangeScan & obs){

	bool	thereIsObservation,hardError;

	laser.doProcessSimple( thereIsObservation, obs, hardError );

	if (hardError){
		//printf("[TEST] Hardware error=true!!\n");
		return false;
	}

	if (!thereIsObservation){
		return false;
	}

	return true;
}


/*-------------------------------------------------------------
				   	  saveObservation
-------------------------------------------------------------*/

double CPtuHokuyo::saveObservation(const char &axis, const int &n_mean){

	double j=0;
	vector<CObservation2DRangeScan>		vObsAux;
	CObservation2DRangeScan obs;

	// Empty laser buffer for obtain the n_mean latest observations
	laser.purgeBuffers();

	// Obtain n_mean observations
	int count=0;
	while(count<n_mean){
		while(!obtainObs(obs)){
			mrpt::system::sleep(1);
		}
		vObsAux.push_back(obs);
		count++;
	}

	int length = minLengthVectors(obs,vObsAux);

	for(int i=0; i<length;i++){
		// the final observation value is the mean of the n_mean observations
		obs.scan.at(i)= 0;
		for(int j=0;j<n_mean;j++){
			obs.scan.at(i)+= vObsAux.at(j).scan.at(i);
		}
		obs.scan.at(i)/= n_mean;
		// If any range is invalid appears as invalid in the mean
		for(int j=0;j<n_mean;j++){
			obs.validRange.at(i) = obs.validRange.at(i)||vObsAux.at(j).validRange.at(i);
		}
	}

	// Obtain actual radians
	ptu->absPosQ(axis,j);

	calculateSensorPose(axis,j,obs);

	// Save observation in a vector for later use

	vObs.push_back(obs);

	return -j;

}


/*-------------------------------------------------------------
					thread_saveObs
-------------------------------------------------------------*/

void thread_saveObs(void *param){

	mrpt::hwdrivers::ThreadParams* t_params = (mrpt::hwdrivers::ThreadParams*) param;
	char axis = t_params->axis;
	const double hokuyo_frec = 0.025;

	CObservation2DRangeScan obs;
	//double j=0;

	while(true){
		if(!t_params->start_capture) break;

		// Obtain a valid observation
		while(!t_params->ptu_hokuyo->obtainObs(obs))
		{
			mrpt::system::sleep(1);
		}

		// Variation of delta pitch while ptu is moving and hokuyo is scanning
		if((axis=='T')||(axis=='t'))
			obs.deltaPitch = (t_params->scan_vel * hokuyo_frec);

		// Obtain time stamp
		obs.timestamp = now();

		// Save obs in vObs vector
		t_params->ptu_hokuyo->vObs.push_back(obs);

		mrpt::system::sleep(1);
	}
}

/*-------------------------------------------------------------
						singleScan
-------------------------------------------------------------*/

bool CPtuHokuyo::singleScan(const char &axis, const int &tWait, const int &movements, const double &radPre, const int &n_mean){

	for(int i=0;i<movements;i++)
	{
		// Save observation
		saveObservation(axis,n_mean);

		// Move PTU
		ptu->moveToOffPos(axis,radPre);

		mrpt::system::sleep(tWait);
	}

	// Save the last observation
	saveObservation(axis,n_mean);

	return true;
}




/*-------------------------------------------------------------
					continuousScan
-------------------------------------------------------------*/

bool CPtuHokuyo::continuousScan(char &axis, const double &velocity, double &initial, double &final){

	double last_position;

	if (!m_ptu_type)
	{
		if(initial<final){
			double aux;
			aux=initial;
			initial = final;
			final = aux;
		}

		const double defSpeed = 0.897598; // Default speed

		// Restart speed
		ptu->speed(axis,defSpeed);
		ptu->aWait();

		// Move PTU to initial position
		ptu->moveToAbsPos(axis,initial);
		ptu->aWait();

		mrpt::system::sleep(2000);

		// Calculate final position in radians
		int initialPos = ptu->radToPos(axis,initial);
		double initialRad = ptu->posToRad(axis,(long) initialPos);
		int finalPos = ptu->radToPos(axis,final);
		double finalRad = ptu->posToRad(axis, (long) finalPos);

		ptu->speed(axis,velocity);
		ptu->aWait();

		// Move PTU to final position
		ptu->moveToAbsPos(axis,final);

		// Struct for save observations thread
		ThreadParams t_params;
		t_params.axis = axis;
		t_params.start_capture = true;
		t_params.scan_vel = velocity;
		t_params.ptu_hokuyo = this;

		// Create save observations thread
		TThreadHandle my_thread = createThread( thread_saveObs, static_cast<void *>(&t_params) );

		last_position = initialRad;
		while(last_position>finalRad){
			// Obtain PTU position
			ptu->absPosQ(axis,last_position);
			// Insert position and time stamp in v_my_pos vector
			my_pos m_pos;
			m_pos.pos = last_position;
			m_pos.timeStamp = now();

			v_my_pos.push_back(m_pos);

			mrpt::system::sleep(1);
		}

		// Stop and destroy thread
		t_params.start_capture = false;
		joinThread(my_thread);

		// Restore default speed
		ptu->speed(axis,defSpeed);
		ptu->aWait();

		// Return to initial position
		ptu->moveToAbsPos(axis,0);

		// Create vObs
		refineVObs(axis);
	}
	else if ( m_ptu_type == 1 ) // TU MICOS
	{
		if ( initial > final )
		{
			double aux = final;
			final = initial;
			initial = aux;
		}

		const double defSpeed = 5; // Default speed
		ptu->speed(0,DEG2RAD(defSpeed));

		ptu->reset();

		double status;
		ptu->status(status);

		// Wait to end of movement
		while(status) {
			mrpt::system::sleep(2);
			ptu->status(status);
		}

		ptu->moveToAbsPos(m_axis,initial);

		ptu->status(status);
		while(status) {
			mrpt::system::sleep(2);
			ptu->status(status);
		}

		// Start PTU movement to final position
		ptu->speed(m_axis,DEG2RAD(velocity));
		ptu->moveToAbsPos(m_axis,final);

		// Struct for save observations thread
		ThreadParams t_params;
		t_params.axis = 'T';
		t_params.start_capture = true;
		t_params.scan_vel = velocity;
		t_params.ptu_hokuyo = this;

		// Create save observations thread
		TThreadHandle my_thread = createThread( thread_saveObs, static_cast<void *>(&t_params) );

		ptu->status(status);
		while(status){
			// Obtain PTU position
			ptu->absPosQ(axis,last_position);
			// Insert position and time stamp in v_my_pos vector
			my_pos m_pos;
			m_pos.pos = last_position;
			m_pos.timeStamp = now();

			v_my_pos.push_back(m_pos);

			mrpt::system::sleep(1);
			ptu->status(status);
		}

		// Stop and destroy thread
		t_params.start_capture = false;
		joinThread(my_thread);

		// Restore default speed
		ptu->speed(axis,DEG2RAD(defSpeed));

		// Create vObs
		refineVObs('T');

	}

	return true;

}

/*-------------------------------------------------------------
					     refineVObs
-------------------------------------------------------------*/

void CPtuHokuyo::refineVObs(const char &axis){

	my_pos m_pos = v_my_pos.at(0);
	CObservation2DRangeScan obs = vObs.at(0);
	unsigned int j=0; // v_my_pos and vObsAux index
	vector<CObservation2DRangeScan> vObsAux;

	// Insert the first observation in vObsAux
	calculateSensorPose(axis,m_pos.pos,obs);

	vObsAux.push_back(obs);

	// Obtain time difference between actual position and next
	double time_b = timeDifference(m_pos.timeStamp,v_my_pos.at(j).timeStamp);
	TTimeStamp t_between_movs = secondsToTimestamp (time_b);

	for(unsigned int i=1; i<vObs.size();i++){
		obs = vObs.at(i);
		// If obs time was pre-intermediate between the two positions
		if(obs.timestamp <= (m_pos.timeStamp + t_between_movs/2)){
			// Obtain min length vectors and performs a mean
			int length = minLengthVectors(vObsAux.at(j),obs,1);
			for(int k=0; k < length ;k++){
				vObsAux.at(j).scan.at(k)= (vObsAux.at(j).scan.at(k)+obs.scan.at(k))/2;
			}
			// See if any range is invalid
			length = minLengthVectors(vObsAux.at(j),obs,0);
			for(int k=0;k< length ;k++){
				vObsAux.at(j).validRange.at(k) = vObsAux.at(j).validRange.at(k)|| obs.validRange.at(k);
			}
		}else{
			if(j<v_my_pos.size()-1){
				// Obtain time difference between actual position and next
				time_b = timeDifference(m_pos.timeStamp,v_my_pos.at(j+1).timeStamp);
				t_between_movs = secondsToTimestamp(time_b);

				// Move to next position
				m_pos = v_my_pos.at(j);
				j++;

				// Calculate sensor pose and insert en vObsAux
				calculateSensorPose(axis,m_pos.pos,obs);

				vObsAux.push_back(obs);

			//If is the last position
			}else if(j==v_my_pos.size()-1){

				t_between_movs = secondsToTimestamp(0);
				m_pos = v_my_pos.at(j);
				j++;

				calculateSensorPose(axis,m_pos.pos,obs);

				vObsAux.push_back(obs);
			}
		}
	}

	vObs.erase( vObs.begin(), vObs.end() );
	vObs = vObsAux;

}

/*-------------------------------------------------------------
					   saveVObs2File
-------------------------------------------------------------*/

bool CPtuHokuyo::saveVObs2File(char *fname){

	CFileOutputStream file;

	// Open data file
	file.open(fname);

	if(!file.fileOpenCorrectly()){
		printf("[TEST] Open file failed!\n");
		return false;
	}

	for(unsigned int i=0;i<vObs.size();i++){
		file << vObs.at(i);
	}
	return true;
}

/*-------------------------------------------------------------
					   saveVObs2File
-------------------------------------------------------------*/

bool CPtuHokuyo::savePitchAndDistances2File(){

	ofstream filePitch;
	ofstream filePoints;

	filePitch.open("Pitch.txt");
	filePoints.open("Distances.txt");

	if(!filePitch.is_open()){
		printf("[TEST] Open file failed!\n");
		return false;
	}

	if(!filePoints.is_open()){
		printf("[TEST] Open file failed!\n");
		return false;
	}

	for(unsigned int i=0;i<vObs.size();i++)
	{
		char cadena[300];

		double pitch, yaw, roll;
		vObs.at(i).sensorPose.getYawPitchRoll(yaw, pitch, roll);

		sprintf(cadena,"%f\t%f\t%f\n", yaw, pitch, roll);
		filePitch << cadena;

		for (unsigned int j=0; j < vObs.at(i).scan.size(); j++)
		{
			char cadena2[300];
			sprintf(cadena2,"%f\t", vObs.at(i).scan.at(j));
			filePoints << cadena2;
		}
		char cadena2[5];
		sprintf(cadena2,"\n");
		filePoints << cadena2;

	}

	filePitch.close();
	filePoints.close();

	printf("[INFO] Save Pitchs to file COMPLETE\n");
	printf("[INFO] Save Distances to file COMPLETE\n");

	return true;
}

/*-------------------------------------------------------------
					  minLengthVectors
-------------------------------------------------------------*/

int CPtuHokuyo::minLengthVectors(CObservation2DRangeScan &obs, vector<CObservation2DRangeScan> &vObsAux){

	unsigned int min= obs.scan.size();

	for(unsigned int i=0;i<vObsAux.size();i++){
		if(vObsAux.at(i).scan.size() < min)
			min = vObsAux.at(i).scan.size();
	}

	return  min;

}



/*-------------------------------------------------------------
				 minLengthVectors
-------------------------------------------------------------*/

int CPtuHokuyo::minLengthVectors(mrpt::slam::CObservation2DRangeScan &obs1, mrpt::slam::CObservation2DRangeScan &obs2, const int &mode){

	if(mode){
		if (obs1.scan.size()>=obs2.scan.size()){
			return obs1.scan.size();
		}else{
			return obs2.scan.size();
		}
	}else{
		if (obs1.validRange.size()>=obs2.validRange.size()){
			return obs1.validRange.size();
		}else{
			return obs2.validRange.size();
		}
	}

}


/*-------------------------------------------------------------
				    calculateSensorPose
-------------------------------------------------------------*/

void CPtuHokuyo::calculateSensorPose(const char &axis, const double &pos, mrpt::slam::CObservation2DRangeScan &obs){

	if (!m_ptu_type) {

		if((axis=='T')||(axis=='t')){
			// Calculate sensor position
			double x=high*cos(pos+DEG2RAD(90.0));
			double z=high*sin(pos+DEG2RAD(90.0));


			obs.sensorPose = CPose3D(x,0,z,0,-pos);

		}else{ // Pan movement

			obs.sensorPose = CPose3D(0,0,high,-pos);
		}
	} else if (m_ptu_type == 1) {

		double pitch=pos-DEG2RAD(34.0);
		double x=0.007+high*sin(pitch);
		double z=high*cos(pitch);

		//double x = 0.007 + high*cos(DEG2RAD(124)-pos);
		//double z = high*sin(DEG2RAD(124)-pos);

		//double x = 0.007 + high * cos(DEG2RAD(50.0) + pos );
		//double z = high * sin(DEG2RAD(50.0) + pos );

		//double pitch = DEG2RAD(40.0) - pos;

		obs.sensorPose = CPose3D(x, 0, z, 0, pitch);

		//cout << endl << "X: " << x << "  Z: " << z << "  Pitch: " << pitch ;
		//cout   << "pos: " << pos << "  pos2: " << RAD2DEG(m_final-pos) << "  initial: " << m_initial << "  final: " << m_final <<  endl;

	}
}


/*-------------------------------------------------------------
						obsPosition
-------------------------------------------------------------*/

int CPtuHokuyo::obsPosition()
{
	//Chek if we have minimun 2 positions
	if(v_ptu_pos.size()<2)
	{
		return 0;
	}
	else
	{
		// Number of observations processed
		double num_obs = 0;
		// Obtain the time of the last ptu position
		double last_time = v_ptu_time.at(v_ptu_time.size()-1);

		for(unsigned int i=0; i < vObs.size(); i++)
		{
			double ts = timestampTotime_t(vObs.at(i).timestamp);
			// Check
			if(ts <= last_time){
				double pos;
				// Calculate laser position
				pos = interpolate2points(ts,v_ptu_time.at(0),v_ptu_pos.at(0),v_ptu_time.at(v_ptu_time.size()-1),v_ptu_pos.at(v_ptu_pos.size()-1));
				calculateSensorPose(m_axis,pos,vObs.at(i));
				num_obs++;
			}
		}

		//Delete until last position
		v_ptu_pos.erase(v_ptu_pos.begin(),v_ptu_pos.end()-1);
		v_ptu_time.erase(v_ptu_time.begin(),v_ptu_time.end()-1);

		return num_obs;

	}

}


/*-------------------------------------------------------------
				         loadConfig_sensorSpecific
-------------------------------------------------------------*/

void  CPtuHokuyo::loadConfig_sensorSpecific(const mrpt::utils::CConfigFileBase &configSource, const std::string &section ){

	m_velocity		= configSource.read_double(section,"velocity",m_velocity);
	m_initial   	= configSource.read_double(section,"initial_pos",m_initial);
	m_final			= configSource.read_double(section,"final_pos",m_final);
	m_hokuyo_frec	= configSource.read_double(section,"hokuyo_frec",m_hokuyo_frec);
	high			= configSource.read_double(section,"high_ptuHokuyo",high);
	m_ptu_type		= configSource.read_int(section,"ptu_type",m_ptu_type);

	#ifdef MRPT_OS_WINDOWS
		laser.setSerialPort( configSource.read_string(section, "COM_hokuyo_port_WIN", laser.getSerialPort(), true ) );
		m_ptu_port = configSource.read_string(section, "COM_ptu_port_WIN", m_ptu_port, true );
	#else
		laser.setSerialPort( configSource.read_string(section, "COM_hokuyo_port_LIN", laser.getSerialPort(), true ) );
		m_ptu_port = configSource.read_string(section, "COM_ptu_port_LIN", m_ptu_port, true );
	#endif
}


/*-------------------------------------------------------------
				        initialize
-------------------------------------------------------------*/

void CPtuHokuyo::initialize(){

	init(m_ptu_port,laser.getSerialPort());

	// Convert initial and final degrees (position) to radians
	m_initial = DEG2RAD(m_initial);
	m_final = DEG2RAD(m_final);

	// Check initial and final positions
	if ( ((!m_ptu_type) && (m_initial < m_final)) || ((m_ptu_type==1) && (m_initial > m_final)) ) {

		double aux;
		aux = m_initial;
		m_initial = m_final;
		m_final = aux;

	}

	if (!m_ptu_type) {

		const double defSpeed = 0.897598; // Default speed

		// Restart speed
		ptu->speed(m_axis,defSpeed);
		ptu->aWait();

		// Move PTU to initial position
		ptu->moveToAbsPos(m_axis,m_initial);
		ptu->aWait();
		mrpt::system::sleep(2000);


		// Calculate exact initial and final position in radians

		int initialPos = ptu->radToPos(m_axis,m_initial);
		m_initial = ptu->posToRad(m_axis,(long) initialPos);
		int finalPos = ptu->radToPos(m_axis,m_final);
		m_final = ptu->posToRad(m_axis, (long) finalPos);


		// Set speed

		ptu->speed(m_axis,m_velocity);
		ptu->aWait();

		// Start PTU movement to final position
		ptu->moveToAbsPos(m_axis,m_final);


	} else if (m_ptu_type == 1) {

		const double defSpeed = 26; // Default speed
		ptu->speed(0,DEG2RAD(defSpeed));

		ptu->reset();

		double status;
		ptu->status(status);

		// Wait to end of movement
		while(status) {
			mrpt::system::sleep(2);
			ptu->status(status);
		}

		// Change motion dir
		//ptu->changeMotionDir();


		// Start PTU movement to final position
		//ptu->moveToAbsPos(m_axis,m_final);

		// Start PTU movement to initial position
		ptu->moveToAbsPos(m_axis,m_initial);

		ptu->status(status);
		while(status) {
			mrpt::system::sleep(2);
			ptu->status(status);
		}

		// Change motion dir
		//ptu->changeMotionDir();

		// Start PTU movement to initial position
		//m_initial = m_final + (m_final - m_initial);
		//ptu->speed(m_axis,DEG2RAD(m_velocity));
		//ptu->moveToAbsPos(m_axis,m_initial);

		// Start PTU movement to final position
		ptu->speed(m_axis,DEG2RAD(m_velocity));
		ptu->moveToAbsPos(m_axis,m_final);

	}

}


/*-------------------------------------------------------------
				          doProcess
-------------------------------------------------------------*/

void CPtuHokuyo::doProcess(){

	double last_position;
	CObservation2DRangeScan obs;

	// Query actual position and save it
	while(!ptu->absPosQ(m_axis,last_position));
		mrpt::system::sleep(100);

	v_ptu_pos.push_back(last_position);
	v_ptu_time.push_back(timestampTotime_t(now()));

	//cout << "Position " << RAD2DEG(last_position) << endl;

	// Number of observations to obtain
	int n_obs = 2;
	// Observations obtained
	int obs_ob = 0;

	// Empty laser buffer
	laser.purgeBuffers();

	// Obtain n_obs valids observations
	while(obs_ob < n_obs)
	{
		if(obtainObs(obs))
		{
			// Variation of delta pitch while ptu is moving and hokuyo is scanning
			if (!m_ptu_type) {
				obs.deltaPitch = (m_velocity * m_hokuyo_frec);
			} else if (m_ptu_type == 1) {
				obs.deltaPitch = (DEG2RAD(m_velocity) * m_hokuyo_frec);
			}

			// Obtain time stamp
			obs.timestamp = now();
			// Save observation in vObs vector
			vObs.push_back(obs);

			++obs_ob;
		}
	}

	//cout << endl << last_position << " " << m_initial << " " << m_final << endl;
	// Chek if we have to change the movement direction
	if (!m_ptu_type) {

		if(last_position <= m_final)
		{
			ptu->moveToAbsPos(m_axis,m_initial);
		}
		else if(last_position >= m_initial)
		{
			ptu->moveToAbsPos(m_axis,m_final);
		}

	} else if (m_ptu_type == 1) {

		double status;
		ptu->status(status);

		//cout << endl << "Status: " << status << "Last position: " << last_position << "Initial: " << m_initial << "Final: " << m_final << endl;

		if(!status && last_position >= m_final-0.1) // Final position gain
		{
			ptu->moveToAbsPos(m_axis,m_initial);
		}
		else if (!status && last_position <= m_initial+0.1) // Initial position gain
		{
			ptu->moveToAbsPos(m_axis,m_final);
		}

	}

	// Obtain the number of observations to process and caculate their positions
	unsigned int num_obs = obsPosition();

	//cout << "Number of observations" << num_obs << endl;

	for(unsigned int i=0; i < num_obs ; i++)
	{
		// Insert processed observation
		appendObservation(CObservation2DRangeScanPtr(new CObservation2DRangeScan (vObs.at(i))));
	}



	// Delete processed observations
	if(vObs.size() == num_obs)
	{
		vObs.clear();
	}
	else
	{
		vObs.erase( vObs.begin(), vObs.end()- (vObs.size()-num_obs));
	}


}
