/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

#include <mrpt/slam.h>  // Precompiled header

#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>

using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::utils;


/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CMetricMapBuilder::CMetricMapBuilder() :
	critZoneChangingMap(),
	options()
{
}

/*---------------------------------------------------------------
					Destructor
  ---------------------------------------------------------------*/
CMetricMapBuilder::~CMetricMapBuilder()
{
}

/*---------------------------------------------------------------
					clear
Clear all elements of the maps, and reset localization to (0,0,0deg).
  ---------------------------------------------------------------*/
void  CMetricMapBuilder::clear()
{
	CSimpleMap			dummyMap;
	CPosePDFGaussian				dummyPose;

	// Initialize with an empty map and pose to (0,0,0)
	initialize( dummyMap, &dummyPose );
}


/*---------------------------------------------------------------
					loadCurrentMapFromFile
  ---------------------------------------------------------------*/
void  CMetricMapBuilder::loadCurrentMapFromFile(const std::string &fileName)
{
	CSimpleMap		map;

	// New file??
	if ( mrpt::system::fileExists( fileName ) )
	{
		std::cout << "[CMetricMapBuilder::loadCurrentMapFromFile] Loading current map from '" << fileName << "' ..." << std::endl;
		CFileGZInputStream		f( fileName);

		// Load from file:
		f >> map;
	}
	else
	{	// Is a new file, start with an empty map:
		std::cout << "[CMetricMapBuilder::loadCurrentMapFromFile] Starting new empty map, since file was not found:'" << fileName << "' ..." << std::endl;
		map.clear();
	}

	// Initialize the map builder with this map
	initialize(map);
}

/*---------------------------------------------------------------
					saveCurrentMapToFile
  ---------------------------------------------------------------*/
void  CMetricMapBuilder::saveCurrentMapToFile(const std::string &fileName, bool compressGZ) const
{
	// get current map:
	CSimpleMap		curmap;
	getCurrentlyBuiltMap(curmap);

	// Save to file:
	if (compressGZ)
		CFileGZOutputStream(fileName) << curmap;
	else
		CFileOutputStream(fileName) << curmap;
}
