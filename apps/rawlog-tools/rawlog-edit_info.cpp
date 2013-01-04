/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;

struct TInfoPerSensorLabel
{
	TInfoPerSensorLabel() : occurrences(0), tim_first(INVALID_TIMESTAMP),tim_last(INVALID_TIMESTAMP) {}

	string		className;
	size_t		occurrences;
	TTimeStamp  tim_first, tim_last;
};

// ======================================================================
//		op_info
// ======================================================================
DECLARE_OP_FUNCTION(op_info)
{
	// A class to do this operation:
	class CRawlogProcessor_Info : public CRawlogProcessor
	{
	public:
		// Stats to gather:
		bool  has_actSF_format;
		bool  has_obs_format;
		size_t  nActions;
		size_t  nSFs;
		map<string,TInfoPerSensorLabel>   infoPerSensorLabel;

		CRawlogProcessor_Info(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) : CRawlogProcessor(in_rawlog,cmdline,verbose)
		{
			has_actSF_format = false;
			has_obs_format   = false;
			nActions = 0;
			nSFs     = 0;
		}

		virtual bool processOneEntry(
			CActionCollectionPtr &actions,
			CSensoryFramePtr     &SF,
			CObservationPtr      &obs)
		{
			// Rawlog format: Normally only one of both should exist simultaneously!
			if (actions || SF) has_actSF_format = true;
			if (obs) has_obs_format = true;
			if (actions) nActions++;
			if (SF) nSFs++;

			// Process each observation individually, either from "obs" or each within a "SF":
			for (size_t idxObs=0; true; idxObs++)
			{
				CObservationPtr  obs_indiv;
				if (obs)
				{
					if (idxObs>0)  break;
					obs_indiv = obs;
				}
				else if (SF)
				{
					if (idxObs>=SF->size()) break;
					obs_indiv = SF->getObservationByIndex(idxObs);
				}
				else break; // shouldn't...

				// Process "obs_indiv":
				ASSERT_(obs_indiv)
				TInfoPerSensorLabel &d = infoPerSensorLabel[obs_indiv->sensorLabel];

				d.className = obs_indiv->GetRuntimeClass()->className;
				d.occurrences++;
				if (d.tim_first==INVALID_TIMESTAMP)
					d.tim_first = obs_indiv->timestamp;
				d.tim_last = obs_indiv->timestamp;
			}

			// Clear read objects:
			actions.clear_unique();
			SF.clear_unique();
			obs.clear_unique();

			return true; // No error.
		}

	}; // end CRawlogProcessor_Info

	// Process
	// ---------------------------------
	CRawlogProcessor_Info proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();


	// Dump statistics:
	// ---------------------------------
	cout << "Time to parse file (sec)          : " << proc.m_timToParse << "\n";
	cout << "Physical file size                : " << mrpt::system::unitsFormat(proc.m_filSize) << "B\n";
	cout << "Uncompressed file size            : " << mrpt::system::unitsFormat(in_rawlog.getPosition()) << "B\n";
	cout << "Compression ratio                 : " << format("%.02f%%\n", 100.0*double(proc.m_filSize)/double(in_rawlog.getPosition()));
	cout << "Overall number of objects         : " << proc.m_rawlogEntry << "\n";
	cout << "Actions/SensoryFrame format       : " << (proc.has_actSF_format ? "Yes":"No") << "\n";
	cout << "Observations format               : " << (proc.has_obs_format ? "Yes":"No") << "\n";

	// By sensor labels:
	cout << "All sensor labels                 : ";
	for (map<string,TInfoPerSensorLabel>::const_iterator it=proc.infoPerSensorLabel.begin();it!=proc.infoPerSensorLabel.end();++it)
	{
		if (it!=proc.infoPerSensorLabel.begin()) cout << ", ";
		cout << it->first;
	}
	cout << "\n";

	for (map<std::string,TInfoPerSensorLabel>::const_iterator it=proc.infoPerSensorLabel.begin();it!=proc.infoPerSensorLabel.end();++it)
	{
		const TTimeStamp	tf = it->second.tim_first;
		const TTimeStamp	tl = it->second.tim_last;
		double Hz = 0, dur = 0;
		if (tf!=INVALID_TIMESTAMP && tl!=INVALID_TIMESTAMP)
		{
			dur = mrpt::system::timeDifference(tf,tl);
			Hz = double(it->second.occurrences>1 ? it->second.occurrences-1 : 1)/dur;
		}
		cout << "Sensor (Label/Occurs/Rate/Durat.) : " <<
			format("%15s /%7u /%5.03f /%.03f\n",
				it->first.c_str(),
				(unsigned)it->second.occurrences,
				Hz,
				dur);
	}

}
