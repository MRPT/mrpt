/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"
#include <mrpt/utils/CConfigFile.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;


// ======================================================================
//		op_sensors_pose
// ======================================================================
DECLARE_OP_FUNCTION(op_sensors_pose)
{
	// A class to do this operation:
	class CRawlogProcessor_SensorsPose : public CRawlogProcessorOnEachObservation
	{
	protected:
		TOutputRawlogCreator	outrawlog;

		typedef mrpt::aligned_containers<std::string,mrpt::poses::CPose3D>::map_t TSensor2PoseMap;
		TSensor2PoseMap desiredSensorPoses;

	public:
		size_t  m_changedPoses;

		CRawlogProcessor_SensorsPose(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			m_changedPoses = 0;

			// Load .ini file with poses:
			string ini_poses;
			getArgValue<string>(cmdline,"sensors-pose",ini_poses);
			ini_poses = trim(ini_poses);
			if (ini_poses.empty())throw std::runtime_error("--sensors-pose op: missing argument: config file to load");
			if (!fileExists(ini_poses)) throw std::runtime_error(string("--sensors-pose op: config file can't be open:")+ini_poses);

			// Load the "ini-file" from the text control:
			CConfigFile  cfg( ini_poses );

			// make a list  "sensor_label -> sensor_pose" by parsing the ini-file:

			vector_string	sections;
			cfg.getAllSections( sections );

			for (vector_string::iterator it=sections.begin();it!=sections.end();++it)
			{
				if (it->empty()) continue;

				// Get sensor label:
				string label = cfg.read_string(*it,"sensorLabel","");
				if (label.empty()) continue;

				CPose3D  the_pose(
					cfg.read_double(*it,"pose_x",0,true),
					cfg.read_double(*it,"pose_y",0,true),
					cfg.read_double(*it,"pose_z",0,true),
					DEG2RAD( cfg.read_double(*it,"pose_yaw",0 ) ),
					DEG2RAD( cfg.read_double(*it,"pose_pitch",0 ) ),
					DEG2RAD( cfg.read_double(*it,"pose_roll",0 ) ) );

				// insert:
				desiredSensorPoses[label] = the_pose;
			} // end for sections

			if (desiredSensorPoses.empty())
				throw std::runtime_error(string("No valid 'sensorLabel' entry was found in ")+ini_poses);

		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			// Check the sensor label:
			TSensor2PoseMap::iterator i = desiredSensorPoses.find(obs->sensorLabel);
			if (i!=desiredSensorPoses.end())
			{
				obs->setSensorPose( i->second );
				m_changedPoses++;
			}
			return true;
		}

		// This method can be reimplemented to save the modified object to an output stream.
		virtual void OnPostProcess(
			mrpt::obs::CActionCollectionPtr &actions,
			mrpt::obs::CSensoryFramePtr     &SF,
			mrpt::obs::CObservationPtr      &obs)
		{
			ASSERT_((actions && SF) || obs)
			if (actions)
					outrawlog.out_rawlog << actions << SF;
			else	outrawlog.out_rawlog << obs;
		}

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_SensorsPose proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of modified entries        : " << proc.m_changedPoses << "\n";

}
