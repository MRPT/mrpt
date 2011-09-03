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

#include <mrpt/slam/CColouredPointsMap.h>
#include <mrpt/slam/CSimplePointsMap.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
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

