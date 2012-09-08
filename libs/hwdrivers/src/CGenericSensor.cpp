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


