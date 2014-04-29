/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

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
