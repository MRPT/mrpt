/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/obs.h>   // Precompiled headers 



#include <mrpt/slam/CObservationGasSensors.h>
using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationGasSensors, CObservation,mrpt::slam)

/** Constructor
 */
CObservationGasSensors::CObservationGasSensors( ) :
	m_readings()
{

}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGasSensors::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 5;
	else
	{
		uint32_t	i,n = m_readings.size();
		out << n;

		for (i=0;i<n;i++)
		{
			out << CPose3D(m_readings[i].eNosePoseOnTheRobot);
			out << m_readings[i].readingsVoltage;
			out << m_readings[i].sensorTypes;
			out << m_readings[i].hasTemperature;
			if (m_readings[i].hasTemperature)
				out << m_readings[i].temperature;
		}

		out << sensorLabel
		    << timestamp;

	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationGasSensors::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 2:
	case 3:
	case 4:
	case 5:
		{
		// A general set of e-nose descriptors:
		uint32_t	i,n;

		in >> n;
		m_readings.resize(n);

		CPose3D  aux;

		for (i=0;i<n;i++)
		{

			in >> aux; m_readings[i].eNosePoseOnTheRobot = aux;
			in >> m_readings[i].readingsVoltage;
			in >> m_readings[i].sensorTypes;
			if (version>=3)
			{
				in >> m_readings[i].hasTemperature;
				if (m_readings[i].hasTemperature)
					in >> m_readings[i].temperature;
			}
			else
			{
				m_readings[i].hasTemperature=false;
				m_readings[i].temperature=0;
			}
		}

		if (version>=4)
				in >> sensorLabel;
		else	sensorLabel = "";

		if (version>=5)
				in >> timestamp;
		else 	timestamp = INVALID_TIMESTAMP;


		}
		break;
	case 0:
	case 1:
		{
			TObservationENose		eNose;

			m_readings.clear();

			// There was a single set of 16 values from "Sancho" (DEC-2006)
			vector_float		readings;
			in >> readings;

			ASSERT_(readings.size()==16);

			// There was TWO e-noses:
			// (1)
			eNose.eNosePoseOnTheRobot = CPose3D( 0.20f,-0.15f, 0.10f ); // (x,y,z) only
			eNose.readingsVoltage.resize(4);
			eNose.readingsVoltage[0] = readings[2];
			eNose.readingsVoltage[1] = readings[4];
			eNose.readingsVoltage[2] = readings[5];
			eNose.readingsVoltage[3] = readings[6];

			eNose.sensorTypes.clear();
			eNose.sensorTypes.resize(4,0);
			m_readings.push_back(eNose);

			// (2)
			eNose.eNosePoseOnTheRobot = CPose3D( 0.20f, 0.15f, 0.10f ); // (x,y,z) only
			eNose.readingsVoltage.resize(4);
			eNose.readingsVoltage[0] = readings[8];
			eNose.readingsVoltage[1] = readings[10];
			eNose.readingsVoltage[2] = readings[12];
			eNose.readingsVoltage[3] = readings[14];

			eNose.sensorTypes.clear();
			eNose.sensorTypes.resize(4,0);
			m_readings.push_back(eNose);

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}


/*---------------------------------------------------------------
                     getSensorPose
 ---------------------------------------------------------------*/
void CObservationGasSensors::getSensorPose( CPose3D &out_sensorPose ) const
{
	if (m_readings.size())
		out_sensorPose = CPose3D(m_readings[0].eNosePoseOnTheRobot);
	else 	out_sensorPose = CPose3D(0,0,0);
}

/*---------------------------------------------------------------
                     setSensorPose
 ---------------------------------------------------------------*/
void CObservationGasSensors::setSensorPose( const CPose3D &newSensorPose )
{
	size_t		i, n = m_readings.size();
	if (n)
		for (i=0;i<n;i++)
			m_readings[i].eNosePoseOnTheRobot= mrpt::math::TPose3D(newSensorPose);
}



