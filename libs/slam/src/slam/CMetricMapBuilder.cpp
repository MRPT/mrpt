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
