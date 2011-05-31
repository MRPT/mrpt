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

#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CBoardIR.h>

using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CBoardIR,mrpt::hwdrivers)

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CBoardIR::CBoardIR( int BUFFER_LENGTH ) :
	m_COM					(),
	m_minRange				(0.1f),
	m_maxRange				(0.8f),
	m_COMname				(),
	m_COMbauds				(4800)
{
	m_sensorLabel = "BoardIR";
}

/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CBoardIR::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
#ifdef MRPT_OS_WINDOWS
	m_COMname = configSource.read_string(iniSection, "COM_port_WIN", m_COMname, true );
#else
	m_COMname = configSource.read_string(iniSection, "COM_port_LIN", m_COMname, true );
#endif

	m_COMbauds		= configSource.read_int( iniSection, "baudRate",m_COMbauds, true );

	std::vector<int32_t>::iterator itIR;
	std::vector<double> aux;				// Auxiliar vector
	for( unsigned int i = 0; i < 6; i++ )
	{
		configSource.read_vector( iniSection, format("pose%d",i), aux, aux, true );	// Get the IR poses
		m_IRPoses[i] = mrpt::poses::TPose3D( aux[0], aux[1], aux[2], DEG2RAD( (float)aux[3]), DEG2RAD( (float)aux[4]), DEG2RAD( (float)aux[5]) );
	}
}


/* -----------------------------------------------------
                Destructor
   ----------------------------------------------------- */
CBoardIR::~CBoardIR()
{
}

/*-------------------------------------------------------------
					getObservation
-------------------------------------------------------------*/
void CBoardIR::getObservation(
	bool							&outThereIsObservation,
	mrpt::slam::CObservationRange	&obs,
	bool							&hardwareError )
{
	outThereIsObservation = false;

	if (!tryToOpenTheCOM())
	{
		m_state = ssError;
		hardwareError = true;
		return;
	}

	hardwareError = false;

	// Read in a loop until a complete frame is read:
	uint8_t  buf[100];
	size_t nTotalRead=0;

	for (;;)
	{
		size_t nToRead;

		if (nTotalRead<=1) nToRead=1;
		else nToRead=(3+2*buf[1])-nTotalRead;

		const size_t bytesRead = m_COM.Read(buf+nTotalRead, nToRead);
		nTotalRead+=bytesRead;

		if (!bytesRead)
		{
			// No more data... skip for now:
			return;
		}
		else if (bytesRead<nToRead)
		{
			// Keep reading:
			continue;
		}


		// Is frame complete?
		if (nTotalRead>=2 && nTotalRead>=(3U+2*buf[1]))
		{
			// Check start & end flags:
			if (buf[0]!=0x69 || buf[nTotalRead-1]!=0x96)
			{
				nTotalRead = 0;
				continue;
			}

			// OK, the frame is valid:
			obs.timestamp				= mrpt::system::getCurrentTime();
			obs.sensorLabel              = m_sensorLabel;
			obs.minSensorDistance		= m_minRange;
			obs.maxSensorDistance		= m_maxRange;
			obs.sensorConeApperture		= DEG2RAD(2.0f);
			obs.sensedData.clear();

			const size_t nSensors = buf[1];

			for( unsigned int j = 0; j < nSensors; j++ )
			{
				mrpt::slam::CObservationRange::TMeasurement obsRange;

				obsRange.sensorID = j;												// The ID of the IR sensor
				obsRange.sensorPose = m_IRPoses[j];									// The pose of the IR sensor
				// 6th degree fitting curve:
				// f(x) = p1*x^6 + p2*x^5 + p3*x^4 + p4*x^3 + p5*x^2 + p6*x + p7 with
				// p1 =       4.088  (-2.889, 11.07)
				// p2 =      -45.35  (-106.2, 15.47)
				// p3 =       208.7  (-1.841, 419.2)
				// p4 =      -512.5  (-880.1, -144.9)
				// p5 =       716.8  (378.4, 1055)
				// p6 =      -560.3  (-714.5, -406.1)
				// p7 =       217.5  (190.7, 244.4)
				unsigned int d = ((unsigned int)buf[2+2*j])+256*((unsigned int)buf[2+2*j+1]);
				double x  = d*(5.0f/1023);						// Conversion of the received value (0-1023) to volts (0-5)
				static const double p1 = 4.088;
				static const double p2 = -45.35;
				static const double p3 = 208.7;
				static const double p4 = -512.5;
				static const double p5 = 716.8;
				static const double p6 = -560.3;
				static const double p7 = 217.5;
				double aux = ( x < 0.3 || x > 2.6 ) ? 0 : 0.01*(p1*pow(x,6)+p2*pow(x,5)+p3*pow(x,4)+p4*pow(x,3)+p5*square(x)+p6*x+p7);
				obsRange.sensedDistance = aux;
				obs.sensedData.push_back( obsRange );								// Put the sensed value within the observation
				//cout << "[" << j << "] : " << "Received: " << dec << (unsigned int)d << endl;
			} // end-for-j

			// OK:
			outThereIsObservation=true;
			return;
		}
		else
		{
			// Check start & end flags:
			if (nTotalRead==1 && buf[0]!=0x69)
				nTotalRead=0; // reset
			else if (nTotalRead==2 && buf[1]>100)
				nTotalRead=0; // reset
		}
	} // end infinite loop

} // end-getobservation

/*-------------------------------------------------------------
					getObservation
-------------------------------------------------------------*/
void CBoardIR::doProcess( )
{
	try
	{
		// Try to connect to the device:
		// Is the COM open?
		if (!tryToOpenTheCOM())
		{
			m_state = ssError;
			THROW_EXCEPTION("Cannot open the serial port");
		}

		mrpt::slam::CObservationRangePtr obs = mrpt::slam::CObservationRange::Create();
		bool thereIsObservation,hardwareError;

		getObservation(thereIsObservation,*obs,hardwareError);

		if (hardwareError)
			THROW_EXCEPTION("Error receiving from IR board.")

		if (thereIsObservation)
			appendObservation( obs );

	} // end-try
	catch(...)
	{
		return;
	}
}

/* -----------------------------------------------------
				setSerialPortName
----------------------------------------------------- */
void CBoardIR::setSerialPortName(const std::string &COM_port)
{
	if (m_COM.isOpen())
		THROW_EXCEPTION("Cannot change serial port name when it's already open")

	m_COMname = COM_port;
}

/* -----------------------------------------------------
				getSerialPortName
----------------------------------------------------- */
std::string CBoardIR::getSerialPortName() const
{
	return m_COMname;
}

/* -----------------------------------------------------
				tryToOpenTheCOM
----------------------------------------------------- */
bool  CBoardIR::tryToOpenTheCOM()
{
	if (m_COM.isOpen())
		return true;	// Already open

	try
	{
		m_COM.open(m_COMname);

		// Config:
		m_COM.setConfig( m_COMbauds, 0,8,1 );
		m_COM.setTimeouts( 100,1,10,1,1 );

		return true; // All OK!
	}
	catch (std::exception &e)
	{
		std::cerr << "[CBoardIR::tryToOpenTheCOM] Error opening or configuring the serial port:" << std::endl << e.what();
		m_COM.close();
		return false;
	}
	catch (...)
	{
		std::cerr << "[CBoardIR::tryToOpenTheCOM] Error opening or configuring the serial port." << std::endl;
		m_COM.close();
		return false;
	}
}
