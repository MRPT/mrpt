/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers
//
#include "rawlog-edit-declarations.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::apps;
using namespace std;
using namespace mrpt::io;

struct TInfoPerSensorLabel
{
	TInfoPerSensorLabel() = default;

	string className;
	size_t occurrences{0};
	TTimeStamp tim_first, tim_last;
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
		bool has_actSF_format = false;
		bool has_obs_format = false;
		size_t nActions = 0;
		size_t nSFs = 0;
		map<string, TInfoPerSensorLabel> infoPerSensorLabel;
		double firstTimestamp = 0;
		double lastTimestamp = 0;

		CRawlogProcessor_Info(
			CFileGZInputStream& in_rawlog, TCLAP::CmdLine& cmdline,
			bool _verbose)
			: CRawlogProcessor(in_rawlog, cmdline, _verbose)
		{
		}

		bool processOneEntry(
			CActionCollection::Ptr& actions, CSensoryFrame::Ptr& SF,
			CObservation::Ptr& obs) override
		{
			// Rawlog format: Normally only one of both should exist
			// simultaneously!
			if (actions || SF) has_actSF_format = true;
			if (obs) has_obs_format = true;
			if (actions) nActions++;
			if (SF) nSFs++;

			// Process each observation individually, either from "obs" or each
			// within a "SF":
			for (size_t idxObs = 0; true; idxObs++)
			{
				CObservation::Ptr obs_indiv;
				if (obs)
				{
					if (idxObs > 0) break;
					obs_indiv = obs;
				}
				else if (SF)
				{
					if (idxObs >= SF->size()) break;
					obs_indiv = SF->getObservationByIndex(idxObs);
				}
				else
					break;	// shouldn't...

				// Process "obs_indiv":
				ASSERT_(obs_indiv);
				TInfoPerSensorLabel& d =
					infoPerSensorLabel[obs_indiv->sensorLabel];

				const auto obsTim = obs_indiv->timestamp;
				if (obsTim != INVALID_TIMESTAMP)
				{
					const double t = mrpt::Clock::toDouble(obsTim);
					if (firstTimestamp == 0 || t < firstTimestamp)
						firstTimestamp = t;

					if (lastTimestamp == 0 || t > lastTimestamp)
						lastTimestamp = t;
				}

				d.className = obs_indiv->GetRuntimeClass()->className;
				d.occurrences++;
				if (d.tim_first == INVALID_TIMESTAMP) d.tim_first = obsTim;
				d.tim_last = obsTim;
			}

			// Clear read objects:
			actions.reset();
			SF.reset();
			obs.reset();

			return true;  // No error.
		}

	};	// end CRawlogProcessor_Info

	// Process
	// ---------------------------------
	CRawlogProcessor_Info proc(in_rawlog, cmdline, verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	cout << "Time to parse file (sec)          : " << proc.m_timToParse << "\n";
	cout << "Physical file size                : "
		 << mrpt::system::unitsFormat(proc.m_filSize) << "B\n";
	cout << "Uncompressed file size            : "
		 << mrpt::system::unitsFormat(in_rawlog.getPosition()) << "B\n";
	cout << "Compression ratio                 : "
		 << format(
				"%.02f%%\n",
				100.0 * double(proc.m_filSize) /
					double(in_rawlog.getPosition()));
	cout << "Overall number of objects         : " << (proc.m_rawlogEntry + 1)
		 << "\n";
	cout << "Actions/SensoryFrame format       : "
		 << (proc.has_actSF_format ? "Yes" : "No") << "\n";
	cout << "Observations format               : "
		 << (proc.has_obs_format ? "Yes" : "No") << "\n";

	cout << "Earliest timestamp                : "
		 << mrpt::format("%.06f", proc.firstTimestamp) << " ("
		 << mrpt::system::dateTimeToString(
				mrpt::Clock::fromDouble(proc.firstTimestamp))
		 << " UTC)\n";

	cout << "Latest timestamp                  : "
		 << mrpt::format("%.06f", proc.lastTimestamp) << " ("
		 << mrpt::system::dateTimeToString(
				mrpt::Clock::fromDouble(proc.lastTimestamp))
		 << " UTC)\n";

	// By sensor labels:
	cout << "All sensor labels                 : ";
	for (auto it = proc.infoPerSensorLabel.begin();
		 it != proc.infoPerSensorLabel.end(); ++it)
	{
		if (it != proc.infoPerSensorLabel.begin()) cout << ", ";
		cout << it->first;
	}
	cout << "\n";

	for (auto it = proc.infoPerSensorLabel.begin();
		 it != proc.infoPerSensorLabel.end(); ++it)
	{
		const TTimeStamp tf = it->second.tim_first;
		const TTimeStamp tl = it->second.tim_last;
		double Hz = 0, dur = 0;
		if (tf != INVALID_TIMESTAMP && tl != INVALID_TIMESTAMP)
		{
			dur = mrpt::system::timeDifference(tf, tl);
			Hz = double(
					 it->second.occurrences > 1 ? it->second.occurrences - 1
												: 1) /
				dur;
		}
		cout << "Sensor (Label/Occurs/Rate/Durat.) : "
			 << format(
					"%15s /%7u /%5.03f /%.03f\n", it->first.c_str(),
					(unsigned)it->second.occurrences, Hz, dur);
	}
}
