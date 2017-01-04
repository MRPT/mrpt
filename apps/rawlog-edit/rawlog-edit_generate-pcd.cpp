/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "rawlog-edit-declarations.h"

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::rawlogtools;
using namespace std;


// ======================================================================
//		op_generate_pcd
// ======================================================================
DECLARE_OP_FUNCTION(op_generate_pcd)
{
	// A class to do this operation:
	class CRawlogProcessor_GeneratePCD : public CRawlogProcessorOnEachObservation
	{
	protected:

	public:
		size_t  entries_done;
		std::string  m_outdir;

		CRawlogProcessor_GeneratePCD(CFileGZInputStream &in_rawlog, TCLAP::CmdLine &cmdline, bool verbose) :
			CRawlogProcessorOnEachObservation(in_rawlog,cmdline,verbose)
		{
			entries_done = 0;
			getArgValue<std::string>(cmdline,"out-dir",  m_outdir);

			if (!mrpt::system::directoryExists(m_outdir))
				throw std::runtime_error(string("ERROR: Output directory does not exist: ")+m_outdir);
		}

		bool processOneObservation(CObservationPtr  &obs)
		{
			const string label_time = format("%s/%06u_%s_%f.pcd",
				m_outdir.c_str(),
				static_cast<unsigned int>(m_rawlogEntry),
				obs->sensorLabel.empty() ? "NOLABEL" : obs->sensorLabel.c_str(),
				timestampTotime_t(obs->timestamp) );
			if (IS_CLASS(obs, CObservation3DRangeScan ) )
			{
				CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
				if (obs3D->hasRangeImage && !obs3D->hasPoints3D)
					obs3D->project3DPointsFromDepthImage();

				if (obs3D->hasPoints3D)
				{
					CColouredPointsMap  map;
					map.insertionOptions.minDistBetweenLaserPoints = 0;

					map.insertObservation(obs3D.pointer());
					if (!map.savePCDFile(label_time,false /* not bin format*/))
						throw std::runtime_error(string("ERROR: While saving file: ")+label_time);
					entries_done++;
				}
			}
			else
			if (IS_CLASS(obs, CObservation2DRangeScan ) )
			{
				CObservation2DRangeScanPtr obs2D = CObservation2DRangeScanPtr(obs);

				CSimplePointsMap map;
				map.insertionOptions.minDistBetweenLaserPoints = 0;

				map.insertObservation(obs2D.pointer());
				if (!map.savePCDFile(label_time,false /* not bin format*/))
					throw std::runtime_error(string("ERROR: While saving file: ")+label_time);
				entries_done++;
			}

			return true;
		}
	};

	// Process
	// ---------------------------------
	CRawlogProcessor_GeneratePCD proc(in_rawlog,cmdline,verbose);
	proc.doProcessRawlog();

	// Dump statistics:
	// ---------------------------------
	VERBOSE_COUT << "Time to process file (sec)        : " << proc.m_timToParse << "\n";
	VERBOSE_COUT << "PCD files generated               : " << proc.entries_done << "\n";

}

