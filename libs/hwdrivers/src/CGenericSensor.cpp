/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CObservation.h>

using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::serialization;
using namespace std;

/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CGenericSensor::CGenericSensor()
	: m_sensorLabel("UNNAMED_SENSOR"),

	  m_path_for_external_images(),
	  m_external_images_format("png")

{
	const char* sVerbose = getenv("MRPT_HWDRIVERS_VERBOSE");
	m_verbose = (sVerbose != nullptr) && atoi(sVerbose) != 0;
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
void CGenericSensor::appendObservations(
	const std::vector<mrpt::serialization::CSerializable::Ptr>& objs)
{
	if (++m_grab_decimation_counter >= m_grab_decimation)
	{
		m_grab_decimation_counter = 0;

		std::lock_guard<std::mutex> lock(m_csObjList);

		for (const auto& obj : objs)
		{
			if (!obj) continue;

			// It must be a CObservation or a CAction!
			TTimeStamp timestamp;

			if (obj->GetRuntimeClass()->derivedFrom(CLASS_ID(CAction)))
			{
				timestamp = dynamic_cast<CAction*>(obj.get())->timestamp;
			}
			else if (obj->GetRuntimeClass()->derivedFrom(
						 CLASS_ID(CObservation)))
			{
				timestamp = dynamic_cast<CObservation*>(obj.get())->timestamp;
			}
			else
				THROW_EXCEPTION("Passed object must be CObservation.");

			// Add it:
			m_objList.insert(TListObsPair(timestamp, obj));
		}
	}
}

/*-------------------------------------------------------------
						getObservations
-------------------------------------------------------------*/
void CGenericSensor::getObservations(TListObservations& lstObjects)
{
	std::lock_guard<std::mutex> lock(m_csObjList);
	lstObjects = m_objList;
	m_objList.clear();  // Memory of objects will be freed by invoker.
}

/*-------------------------------------------------------------

						createSensor

-------------------------------------------------------------*/
CGenericSensor* CGenericSensor::createSensor(const std::string& className)
{
	registered_sensor_classes_t& regs = get_registered_sensor_classes();
	const auto it = regs.find(className);
	return it == regs.end() ? nullptr : it->second->ptrCreateObject();
}

// Singleton
CGenericSensor::registered_sensor_classes_t&
	CGenericSensor::get_registered_sensor_classes()
{
	static registered_sensor_classes_t reg;
	return reg;
}

/*-------------------------------------------------------------
						registerClass
-------------------------------------------------------------*/
void CGenericSensor::registerClass(const TSensorClassId* pNewClass)
{
	registered_sensor_classes_t& regs = get_registered_sensor_classes();
	regs[pNewClass->className] = pNewClass;
}

/** Loads the generic settings common to any sensor (See CGenericSensor), then
 * call to "loadConfig_sensorSpecific"
 *  \exception This method throws an exception with a descriptive message if
 * some critical parameter is missing or has an invalid value.
 */
void CGenericSensor::loadConfig(
	const mrpt::config::CConfigFileBase& cfg, const std::string& sect)
{
	MRPT_START

	m_process_rate = cfg.read_double(
		sect, "process_rate", 0);  // Leave it to 0 so rawlog-grabber can detect
	// if it's not set by the user.
	m_max_queue_len = static_cast<size_t>(
		cfg.read_int(sect, "max_queue_len", int(m_max_queue_len)));
	m_grab_decimation = static_cast<size_t>(
		cfg.read_int(sect, "grab_decimation", int(m_grab_decimation)));

	m_sensorLabel = cfg.read_string(sect, "sensorLabel", m_sensorLabel);

	m_grab_decimation_counter = 0;

	loadConfig_sensorSpecific(cfg, sect);

	MRPT_END
}
