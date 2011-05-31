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

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
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

		std::map<std::string,mrpt::poses::CPose3D>	desiredSensorPoses;

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
			std::map<std::string,mrpt::poses::CPose3D>::iterator i = desiredSensorPoses.find(obs->sensorLabel);
			if (i!=desiredSensorPoses.end())
			{
				obs->setSensorPose( i->second );
				m_changedPoses++;
			}
			return true;
		}

		// This method can be reimplemented to save the modified object to an output stream.
		virtual void OnPostProcess(
			mrpt::slam::CActionCollectionPtr &actions,
			mrpt::slam::CSensoryFramePtr     &SF,
			mrpt::slam::CObservationPtr      &obs)
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

// ======================================================================
//		op_camera_params
// ======================================================================
DECLARE_OP_FUNCTION(op_camera_params)
{
	// A class to do this operation:
	class CRawlogProcessor_CamParams : public CRawlogProcessorOnEachObservation
	{
	protected:
		TOutputRawlogCreator	outrawlog;

		string   target_label;
		mrpt::utils::TCamera    new_cam_params;

	public:
		size_t  m_changedCams;

		CRawlogProcessor_CamParams(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			m_changedCams = 0;

			// Load .ini file with poses:
			string   str;
			getArgValue<string>(cmdline,"camera-params",str);

			vector<string> lstTokens;
			tokenize(str,",",lstTokens);
			if (lstTokens.size()!=2)
				throw std::runtime_error("--camera-params op: argument must be in the format: --camera-params LABEL,file.ini");

			target_label = lstTokens[0];

			const string &fil = lstTokens[1];
			if (!fileExists(fil)) throw std::runtime_error(string("--camera-params op: config file can't be open:")+fil);

			// Load:
			CConfigFile  cfg( fil );
			new_cam_params.loadFromConfigFile("CAMERA_PARAMS",cfg);
		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			if ( strCmpI(obs->sensorLabel,target_label) && IS_CLASS(obs,CObservationImage))
			{
				CObservationImagePtr o = CObservationImagePtr(obs);
				o->cameraParams = new_cam_params;
				m_changedCams++;
			}
			return true;
		}

		// This method can be reimplemented to save the modified object to an output stream.
		virtual void OnPostProcess(
			mrpt::slam::CActionCollectionPtr &actions,
			mrpt::slam::CSensoryFramePtr     &SF,
			mrpt::slam::CObservationPtr      &obs)
		{
			ASSERT_((actions && SF) || obs)
			if (actions)
					outrawlog.out_rawlog << actions << SF;
			else	outrawlog.out_rawlog << obs;
		}

	};

	// Process
	// ---------------------------------
	CRawlogProcessor_CamParams proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "Number of modified entries        : " << proc.m_changedCams << "\n";

}

