/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/config.h>

#include <mrpt/hwdrivers/CActivMediaRobotBase.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservationOdometry.h>

#if MRPT_HAS_ARIA
	#include "Aria.h"
#endif


IMPLEMENTS_GENERIC_SENSOR(CActivMediaRobotBase,mrpt::hwdrivers)

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::hwdrivers;
using namespace std;

#define THE_ROBOT	static_cast<ArRobot*>(m_robot)

/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CActivMediaRobotBase::CActivMediaRobotBase() :
	m_com_port			(),
	m_robotBaud			( 115200 ),
	m_firstIncreOdometry(true),
	m_enableSonars		(false),
	m_robot				(NULL),
	m_sonarDev			(NULL),
	m_simpleConnector	(NULL),
	m_lastTimeSonars	(0),
	m_enableJoyControl 	(false),
    m_joy_max_v			(0.20f),
    m_joy_max_w			(DEG2RAD(20.0f)),
	m_joystick			(),
	m_last_do_process	(INVALID_TIMESTAMP),
	m_capture_rate		(10.0)
{
#if MRPT_HAS_ARIA
	Aria::init();

	m_robot		= (void*)new ArRobot();
	m_sonarDev	= (void*)new ArSonarDevice();

	// add the sonar to the m_robot
    THE_ROBOT->addRangeDevice((ArSonarDevice*)m_sonarDev);

#ifdef MRPT_OS_WINDOWS
	m_com_port = "COM1";
#else
	m_com_port = "/dev/ttyS1";
#endif

#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
						Destructor
-------------------------------------------------------------*/
CActivMediaRobotBase::~CActivMediaRobotBase()
{
#if MRPT_HAS_ARIA
	// Disconnect comms:
	disconnectAndDisableMotors();

	// Destroy object:
	m_simpleConnector = NULL;
	m_robot = NULL;
	m_sonarDev = NULL;

	// Shutdown ARIA threads:
	Aria::shutdown();
	cout << "[~CActivMediaRobotBase] ARIA shutdown done" << endl;
#endif
}

/*-------------------------------------------------------------
						loadConfig
-------------------------------------------------------------*/
void  CActivMediaRobotBase::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
#ifdef MRPT_OS_WINDOWS
	m_com_port = configSource.read_string(iniSection,"robotPort_WIN",m_com_port,true);
#else
	m_com_port = configSource.read_string(iniSection,"robotPort_LIN",m_com_port,true);
#endif
	m_robotBaud = configSource.read_int(iniSection, "robotBaud", m_robotBaud, true );

	m_enableSonars = configSource.read_bool(iniSection, "enableSonars", m_enableSonars );

	m_enableJoyControl = configSource.read_bool(iniSection, "joystick_control", m_enableJoyControl );
	m_joy_max_v = configSource.read_float(iniSection, "joystick_max_v", m_joy_max_v );
	m_joy_max_w = DEG2RAD( configSource.read_float(iniSection, "joystick_max_w_degps", RAD2DEG(m_joy_max_w) ) );
	m_capture_rate = configSource.read_double(iniSection, "capture_rate", m_capture_rate );

}

/*-------------------------------------------------------------
						setSerialPortConfig
-------------------------------------------------------------*/
void  CActivMediaRobotBase::setSerialPortConfig(
	const std::string  &portName,
	int		portBaudRate )
{
	m_com_port = portName;
	m_robotBaud = portBaudRate;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
void CActivMediaRobotBase::initialize()
{
#if MRPT_HAS_ARIA
	char		*args[10];
	int			nArgs;

	char		strBaud[100];
	os::sprintf(strBaud,sizeof(strBaud),"%i", m_robotBaud);

	std::string strCOM = m_com_port;
#ifdef MRPT_OS_WINDOWS
    // Is it COMX, X>4? ->  "\\.\COMX"
    if ( tolower( strCOM[0]) =='c' && tolower( strCOM[1]) =='o' && tolower( strCOM[2]) =='m' )
    {
        // Need to add "\\.\"?
        if (strCOM.size()>4 || strCOM[3]>'4')
			strCOM= std::string("\\\\.\\") + strCOM;
    }
#endif

	char strCOM2[100];
	strcpy( strCOM2, strCOM.c_str() );

	args[0] = (char*)"mrpt";
	args[1] = (char*)"-robotPort";	args[2] = strCOM2;
	args[3] = (char*)"-robotBaud"; args[4] = strBaud;
	args[5] = NULL;
	nArgs = 5;

	if (m_simpleConnector) delete (ArSimpleConnector*)m_simpleConnector;

	m_simpleConnector = new ArSimpleConnector( &nArgs, args );
	static_cast<ArSimpleConnector*>(m_simpleConnector)->parseArgs();

	cout << "[CActivMediaRobotBase::init] Enabling motors..." << endl;
	connectAndEnableMotors();
	cout << "[CActivMediaRobotBase::init] Done" << endl;
	m_state = ssWorking;
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}


/*-------------------------------------------------------------
						connectAndEnableMotors
-------------------------------------------------------------*/
void CActivMediaRobotBase::connectAndEnableMotors()
{
#if MRPT_HAS_ARIA
	// Establecimiento de la conexión con el pioneer
	if (!static_cast<ArSimpleConnector*>(m_simpleConnector)->connectRobot( THE_ROBOT ))
	{
		THROW_EXCEPTION_CUSTOM_MSG1("[CActivMediaRobotBase] Couldn't connect to robot thru %s", m_com_port.c_str() )
	}
	else
	{
		// Enable processing thread:
		THE_ROBOT->lock();
		THE_ROBOT->runAsync(true);
		THE_ROBOT->unlock();

		mrpt::system::sleep(500);

		CTicTac	tictac;
		tictac.Tic();

		if (!THE_ROBOT->areMotorsEnabled())  // Are the motors already enabled?
		{
			THE_ROBOT->lock();
			THE_ROBOT->enableMotors();
			THE_ROBOT->unlock();
			mrpt::system::sleep(500);

			bool	success = false;

			while (tictac.Tac()<4000)
			{
			    THE_ROBOT->lock();
			    if (!THE_ROBOT->isRunning())
				{
					THROW_EXCEPTION("ARIA robot is not running");
				}
				if (THE_ROBOT->areMotorsEnabled())
				{
					THE_ROBOT->unlock();
					success = true;
					break;
				}
				THE_ROBOT->unlock();
				mrpt::system::sleep(100);
			}

			if (!success)
			{
				disconnectAndDisableMotors();
				THROW_EXCEPTION("Couldn't enable robot motors");
			}

			// Start continuous stream of packets:
			THE_ROBOT->lock();
			THE_ROBOT->requestEncoderPackets();

			if (m_enableSonars)
					THE_ROBOT->enableSonar();
			else	THE_ROBOT->disableSonar();

			THE_ROBOT->unlock();

			m_firstIncreOdometry = true;
		}
	}
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
						areMotorsEnabled
-------------------------------------------------------------*/

bool CActivMediaRobotBase::areMotorsEnabled() const
{
#if MRPT_HAS_ARIA
	return THE_ROBOT->areMotorsEnabled();
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}
/*-------------------------------------------------------------
						DisableMotors
-------------------------------------------------------------*/
void CActivMediaRobotBase::DisableMotors()
{
#if MRPT_HAS_ARIA
	THE_ROBOT->lock();
	if (THE_ROBOT->areMotorsEnabled())
	{
			THE_ROBOT->setVel(0.0);		    // Stop motors
			THE_ROBOT->disableMotors();		// Disabling motors
			cout << "[CActivMediaRobotBase] Disabling motors...";
	}
	THE_ROBOT->unlock();
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif

}
/*-------------------------------------------------------------
						EnableMotors
-------------------------------------------------------------*/
void CActivMediaRobotBase::EnableMotors()
{
#if MRPT_HAS_ARIA
	THE_ROBOT->lock();
	if (!THE_ROBOT->areMotorsEnabled())
	{
			THE_ROBOT->enableMotors();		
			cout << "[CActivMediaRobotBase] Enabling motors...";		
	}
	THE_ROBOT->unlock();
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
				disconnectAndDisableMotors
-------------------------------------------------------------*/
void CActivMediaRobotBase::disconnectAndDisableMotors()
{
#if MRPT_HAS_ARIA
	if (!THE_ROBOT) return;

	THE_ROBOT->lock();
	THE_ROBOT->stopEncoderPackets();
	THE_ROBOT->unlock();

	mrpt::system::sleep(1);

	THE_ROBOT->lock();
	if (THE_ROBOT->areMotorsEnabled())
	{
			THE_ROBOT->stopRunning();		// Detiene movimiento
			cout << "[CActivMediaRobotBase] Disabling motors...";
			THE_ROBOT->disableMotors();		// Desactiva los motores
			cout << "Ok" << endl;
	}

	cout << "[CActivMediaRobotBase] Disconnecting...";
	THE_ROBOT->disconnect();
	cout << "Ok" << endl;

	THE_ROBOT->unlock();
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CActivMediaRobotBase::doProcess()
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")

	TTimeStamp tnow = mrpt::system::now();

	const bool do_get_observations = (m_last_do_process==INVALID_TIMESTAMP) || (mrpt::system::timeDifference(m_last_do_process, tnow)>=1.0/m_capture_rate);

	// Collect odometry:
	// ----------------------------
	if (do_get_observations)
	{
		m_last_do_process = tnow;

		int64_t  lticks, rticks;
		CPose2D  odom;
		double vel,w;

		this->getOdometryFull(odom,vel,w,lticks,rticks);

		CObservationOdometryPtr obsOdom = CObservationOdometry::Create();
		obsOdom->sensorLabel = "ACTIVMEDIA_BASE_ODOMETRY";
		obsOdom->timestamp = mrpt::system::now();

		obsOdom->odometry = odom;
		obsOdom->hasEncodersInfo = true;
		obsOdom->encoderLeftTicks = lticks;
		obsOdom->encoderRightTicks = rticks;
		obsOdom->hasVelocities = true;
		obsOdom->velocityLocal.vx = vel;
		obsOdom->velocityLocal.vy = .0;
		obsOdom->velocityLocal.omega = w;

		this->appendObservation(obsOdom);	// Send to the output queue
	}


	// Collect sonar data:
	// ----------------------------
	if (m_enableSonars && do_get_observations)
	{
		bool thereIsObservation;
		CObservationRange  obsSonar;

		getSonarsReadings(thereIsObservation, obsSonar);

		if (thereIsObservation)
		{
			obsSonar.sensorLabel = "ACTIVMEDIA_BASE_SONARS";
			obsSonar.timestamp = mrpt::system::now();

			this->appendObservation( CObservationRangePtr( new  CObservationRange(obsSonar) ) );	// Send to the output queue
		}
	}


    // Control with a Joystick?
	// ----------------------------
    if (m_enableJoyControl)
	{
		float jx,jy,jz;
		vector_bool joy_btns;

		if (m_joystick.getJoystickPosition(0,jx,jy,jz, joy_btns))
		{
			float des_v = - jy * m_joy_max_v;
			float des_w = - jx * m_joy_max_w;

            bool deadman_switch = false;
            for (unsigned int i = 0; i < joy_btns.size(); ++i)
            {
                deadman_switch |= joy_btns[i];
            }
            if (deadman_switch)
            {
                this->setVelocities(des_v,des_w);
            }
            else
            {
                this->setVelocities(0,0);
            }

			static int cnt = 0;
			if (cnt++ == 100)
			{
				cout << "[CActivMediaRobotBase] Joystick control: v=" << des_v << " m/s  w=" << RAD2DEG(des_w) << " deg/s" << endl;
				cnt=0;
			}
		}
		else
		{
			static bool warn_nojoy = false;
			if (!warn_nojoy)
			{
				warn_nojoy = true;
				cerr << endl << endl << "[CActivMediaRobotBase] ***** WARNING: Couldn't access the joystick *****" << endl << endl;
			}
		}
	}

#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					changeOdometry
-------------------------------------------------------------*/
void CActivMediaRobotBase::changeOdometry(const mrpt::poses::CPose2D &newOdometry)
{
#if MRPT_HAS_ARIA
	ArPose	pos_actual;

	THE_ROBOT->lock();
		pos_actual.setPose( newOdometry.x()*1000, newOdometry.y()*1000, RAD2DEG( newOdometry.phi() ) );
		THE_ROBOT->setDeadReconPose( pos_actual );
	THE_ROBOT->unlock();
#else
	MRPT_UNUSED_PARAM(newOdometry);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}


/*-------------------------------------------------------------
					getOdometry
-------------------------------------------------------------*/
void CActivMediaRobotBase::getOdometry(poses::CPose2D &out_odom)
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")
	THE_ROBOT->lock();

	ArPose	pose = THE_ROBOT->getEncoderPose();
	out_odom.x( pose.getX() * 0.001 );
	out_odom.y( pose.getY() * 0.001 );
	out_odom.phi( DEG2RAD( pose.getTh() ) );

	THE_ROBOT->unlock();
#else
	MRPT_UNUSED_PARAM(out_odom);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					getOdometryFull
-------------------------------------------------------------*/
void CActivMediaRobotBase::getOdometryFull(
	poses::CPose2D	&out_odom,
	double			&out_lin_vel,
	double			&out_ang_vel,
	int64_t			&out_left_encoder_ticks,
	int64_t			&out_right_encoder_ticks
	)
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")
	THE_ROBOT->lock();

	// Odometry:
	ArPose	pose = THE_ROBOT->getEncoderPose();
	out_odom.x( pose.getX() * 0.001 );
	out_odom.y( pose.getY() * 0.001 );
	out_odom.phi( DEG2RAD( pose.getTh() ) );

	// Velocities:
	out_lin_vel = THE_ROBOT->getVel() * 0.001;
	out_ang_vel = DEG2RAD( THE_ROBOT->getRotVel() );

	// Encoders:
	out_left_encoder_ticks	= THE_ROBOT->getLeftEncoder();
	out_right_encoder_ticks	= THE_ROBOT->getRightEncoder();

	THE_ROBOT->unlock();
#else
	MRPT_UNUSED_PARAM(out_odom); MRPT_UNUSED_PARAM(out_lin_vel); MRPT_UNUSED_PARAM(out_ang_vel);
	MRPT_UNUSED_PARAM(out_left_encoder_ticks); MRPT_UNUSED_PARAM(out_right_encoder_ticks);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					getOdometryIncrement
-------------------------------------------------------------*/
void CActivMediaRobotBase::getOdometryIncrement(
	poses::CPose2D	&out_incr_odom,
	double			&out_lin_vel,
	double			&out_ang_vel,
	int64_t			&out_incr_left_encoder_ticks,
	int64_t			&out_incr_right_encoder_ticks
	)
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")
	THE_ROBOT->lock();

	static CPose2D	last_pose;
	static int64_t  last_left_ticks=0, last_right_ticks=0;

	CPose2D		cur_pose;
	int64_t		cur_left_ticks, cur_right_ticks;

	// Velocities:
	out_lin_vel = THE_ROBOT->getVel() * 0.001;
	out_ang_vel = DEG2RAD( THE_ROBOT->getRotVel() );

	// Current odometry:
	ArPose	pose = THE_ROBOT->getEncoderPose();
	cur_pose.x( pose.getX() * 0.001 );
	cur_pose.y( pose.getY() * 0.001 );
	cur_pose.phi( DEG2RAD( pose.getTh() ) );

	// Current encoders:
	cur_left_ticks	= THE_ROBOT->getLeftEncoder();
	cur_right_ticks	= THE_ROBOT->getRightEncoder();

	// Compute increment:
	if (m_firstIncreOdometry)
	{
		// First time:
		m_firstIncreOdometry = false;
		out_incr_odom = CPose2D(0,0,0);
		out_incr_left_encoder_ticks  = 0;
		out_incr_right_encoder_ticks = 0;
	}
	else
	{
		// Normal case:
		out_incr_odom = cur_pose - last_pose;
		out_incr_left_encoder_ticks = cur_left_ticks - last_left_ticks;
		out_incr_right_encoder_ticks = cur_right_ticks - last_right_ticks;
	}

	// save for the next time:
	last_pose		 = cur_pose;
	last_left_ticks  = cur_left_ticks;
	last_right_ticks = cur_right_ticks;

	THE_ROBOT->unlock();
#else
	MRPT_UNUSED_PARAM(out_incr_odom); MRPT_UNUSED_PARAM(out_lin_vel); MRPT_UNUSED_PARAM(out_ang_vel);
	MRPT_UNUSED_PARAM(out_incr_left_encoder_ticks); MRPT_UNUSED_PARAM(out_incr_right_encoder_ticks);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					getSonarsReadings
-------------------------------------------------------------*/
void CActivMediaRobotBase::getSonarsReadings( bool &thereIsObservation, CObservationRange	&obs )
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")
	THE_ROBOT->lock();

	obs.minSensorDistance = 0;
	obs.maxSensorDistance = 30;

	int		i,N =THE_ROBOT->getNumSonar();

	obs.sensorLabel = "BASE_SONARS";
	obs.sensorConeApperture = DEG2RAD( 30 );
	obs.timestamp = system::now();

	obs.sensedData.clear();

	unsigned int time_cnt = THE_ROBOT->getCounter();

	if (m_lastTimeSonars == time_cnt)
	{
		thereIsObservation = false;
		THE_ROBOT->unlock();
		return;
	}

	for (i=0;i<N;i++)
	{
		ArSensorReading		*sr = THE_ROBOT->getSonarReading(i);

		if (sr->getIgnoreThisReading()) continue;

//		if (!sr->isNew(time_cnt))
//		{
			//thereIsObservation = false;
			//break;
//		}

		obs.sensedData.push_back( CObservationRange::TMeasurement() );
		CObservationRange::TMeasurement & newObs = obs.sensedData.back();

		newObs.sensorID = i;
		newObs.sensorPose.x = 0.001*sr->getSensorX();
		newObs.sensorPose.y = 0.001*sr->getSensorY();
		newObs.sensorPose.z = 0; //0.001*sr->getSensorZ();
		newObs.sensorPose.yaw = DEG2RAD( sr->getSensorTh() );
		newObs.sensorPose.pitch = 0;
		newObs.sensorPose.roll = 0;

		newObs.sensedDistance = 0.001*THE_ROBOT->getSonarRange(i);
	}
	THE_ROBOT->unlock();

	thereIsObservation = !obs.sensedData.empty();

	// keep the last time:
	if (thereIsObservation)
		m_lastTimeSonars = time_cnt;

#else
	MRPT_UNUSED_PARAM(thereIsObservation); MRPT_UNUSED_PARAM(obs);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					getBatteryCharge
-------------------------------------------------------------*/
void CActivMediaRobotBase::getBatteryCharge( double &out_batery_volts )
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")
	THE_ROBOT->lock();
		out_batery_volts = THE_ROBOT->getBatteryVoltageNow();
	THE_ROBOT->unlock();
#else
	MRPT_UNUSED_PARAM(out_batery_volts);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					getRealBatteryCharge
-------------------------------------------------------------*/
void CActivMediaRobotBase::getRealBatteryCharge( double &out_batery_volts )
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")
	THE_ROBOT->lock();
	out_batery_volts = THE_ROBOT->getRealBatteryVoltage();
	THE_ROBOT->unlock();
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					setVelocities
-------------------------------------------------------------*/
void CActivMediaRobotBase::setVelocities( const double lin_vel, const double ang_vel)
{
#if MRPT_HAS_ARIA
	THE_ROBOT->lock();
		THE_ROBOT->setVel( lin_vel*1000 );
		THE_ROBOT->setRotVel( RAD2DEG( ang_vel ) );
	THE_ROBOT->unlock();
#else
	MRPT_UNUSED_PARAM(lin_vel); MRPT_UNUSED_PARAM(ang_vel);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					enableSonars
-------------------------------------------------------------*/
void CActivMediaRobotBase::enableSonars()
{
#if MRPT_HAS_ARIA
	m_enableSonars = true;
	if (THE_ROBOT)
	{
		THE_ROBOT->lock();
		THE_ROBOT->enableSonar();
		THE_ROBOT->unlock();
	}
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					disableSonars
-------------------------------------------------------------*/
void CActivMediaRobotBase::disableSonars()
{
#if MRPT_HAS_ARIA
	m_enableSonars = false;
	if (THE_ROBOT)
	{
		THE_ROBOT->lock();
		THE_ROBOT->disableSonar();
		THE_ROBOT->unlock();
	}
#else
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}


/*-------------------------------------------------------------
					TRobotDescription
-------------------------------------------------------------*/
CActivMediaRobotBase::TRobotDescription::TRobotDescription() :
	nFrontBumpers	(0),
	nRearBumpers	(0),
	nSonars			(0)
{
}

/*-------------------------------------------------------------
					getRobotInformation
-------------------------------------------------------------*/
void CActivMediaRobotBase::getRobotInformation(CActivMediaRobotBase::TRobotDescription &info)
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")

	THE_ROBOT->lock();

	info.nFrontBumpers = THE_ROBOT->getNumFrontBumpers();
	info.nRearBumpers  = THE_ROBOT->getNumRearBumpers();
	info.nSonars       = THE_ROBOT->getNumSonar();

	THE_ROBOT->unlock();
#else
	MRPT_UNUSED_PARAM(info);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}


/*-------------------------------------------------------------
					getBumpers
-------------------------------------------------------------*/
void CActivMediaRobotBase::getBumpers(vector_bool &bumper_state)
{
#if MRPT_HAS_ARIA
	ASSERTMSG_(THE_ROBOT!=NULL, "Robot is not connected")

	THE_ROBOT->lock();

	int v = THE_ROBOT->getStallValue();
	int Nf = THE_ROBOT->getNumFrontBumpers();
	int Nr = THE_ROBOT->getNumRearBumpers();

	THE_ROBOT->unlock();

	bumper_state.clear();
	for (int i=0;i<Nr;i++)
		bumper_state.push_back(  (v & (1<<(i+1)) )!=0 );
	for (int i=0;i<Nf;i++)
		bumper_state.push_back(  (v & (1<<(i+9)) )!=0 );

#else
	MRPT_UNUSED_PARAM(bumper_state);
	THROW_EXCEPTION("MRPT has been compiled with 'MRPT_BUILD_ARIA'=OFF, so this class cannot be used.");
#endif
}
