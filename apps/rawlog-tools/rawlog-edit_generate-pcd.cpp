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

