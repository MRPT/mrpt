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

#include <mrpt/slam/CAction.h>
#include <mrpt/slam/CObservation.h>

using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace std;

map< std::string , const TSensorClassId *>	CGenericSensor::m_knownClasses;

extern CStartUpClassesRegister  mrpt_hwdrivers_class_reg;
const int dumm = mrpt_hwdrivers_class_reg.do_nothing(); // Avoid compiler removing this class in static linking

/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CGenericSensor::CGenericSensor() :
	m_process_rate(0),
	m_max_queue_len(200),
	m_grab_decimation(0),
	m_sensorLabel("UNNAMED_SENSOR"),
	m_grab_decimation_counter(0),
	m_state( ssInitializing ),
	m_path_for_external_images	(),
	m_external_images_format	("jpg"),
	m_external_images_jpeg_quality (95)
{
}

/*-------------------------------------------------------------
						Destructor
-------------------------------------------------------------*/
CGenericSensor::~CGenericSensor()
{
	// Free objects in list, if any:
	m_objList.clear();
}

/*-------------------------------------------------------------
						appendObservations
-------------------------------------------------------------*/
void CGenericSensor::appendObservations( const std::vector<mrpt::utils::CSerializablePtr> &objs)
{
	if (++m_grab_decimation_counter>=m_grab_decimation)
	{
		m_grab_decimation_counter = 0;

		synch::CCriticalSectionLocker	lock( & m_csObjList );

		for (size_t i=0;i<objs.size();i++)
		{
			const CSerializablePtr &obj = objs[i];
			if (!obj) continue;

			// It must be a CObservation or a CAction!
			TTimeStamp	timestamp;

			if ( obj->GetRuntimeClass()->derivedFrom( CLASS_ID(CAction) ) )
			{
				timestamp = CActionPtr(obj)->timestamp;
			}
			else
			if ( obj->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation) ) )
			{
				timestamp = CObservationPtr(obj)->timestamp;
			}
			else THROW_EXCEPTION("Passed object must be CObservation.");

			// Add it:
			m_objList.insert( TListObsPair(timestamp, obj) );
		}
	}
}


/*-------------------------------------------------------------
						getObservations
-------------------------------------------------------------*/
void CGenericSensor::getObservations( TListObservations	&lstObjects )
{
	synch::CCriticalSectionLocker	lock( & m_csObjList );
	lstObjects = m_objList;
	m_objList.clear();			// Memory of objects will be freed by invoker.
}


extern void registerAllClasses_mrpt_hwdrivers();

/*-------------------------------------------------------------

						createSensor

-------------------------------------------------------------*/
CGenericSensor* CGenericSensor::createSensor(const std::string &className)
{
	registerAllClasses_mrpt_hwdrivers();	// make sure all sensors are registered.

	std::map< std::string , const TSensorClassId *>::iterator it=m_knownClasses.find(className);
	return it==m_knownClasses.end() ? NULL : it->second->ptrCreateObject();
}


/*-------------------------------------------------------------
						registerClass
-------------------------------------------------------------*/
void CGenericSensor::registerClass(const TSensorClassId* pNewClass)
{
	m_knownClasses[ pNewClass->className ] = pNewClass;
}



/** Loads the generic settings common to any sensor (See CGenericSensor), then call to "loadConfig_sensorSpecific"
  *  \exception This method throws an exception with a descriptive message if some critical parameter is missing or has an invalid value.
  */
void  CGenericSensor::loadConfig(
	const mrpt::utils::CConfigFileBase &cfg,
	const std::string			&sect )
{
	MRPT_START

	m_process_rate  = cfg.read_double(sect,"process_rate",0 );  // Leave it to 0 so rawlog-grabber can detect if it's not set by the user.
	m_max_queue_len = static_cast<size_t>(cfg.read_int(sect,"max_queue_len",int(m_max_queue_len)));
	m_grab_decimation = static_cast<size_t>(cfg.read_int(sect,"grab_decimation",int(m_grab_decimation)));

	m_sensorLabel	= cfg.read_string( sect, "sensorLabel", m_sensorLabel );

	m_grab_decimation_counter = 0;

	loadConfig_sensorSpecific(cfg,sect);

	MRPT_END
}


