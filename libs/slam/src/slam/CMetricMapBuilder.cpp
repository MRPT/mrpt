/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CMetricMapBuilder.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::io;
using namespace mrpt::poses;
using namespace mrpt::serialization;

CMetricMapBuilder::CMetricMapBuilder()
	: mrpt::system::COutputLogger("CMetricMapBuilder"),
	  options(this->m_min_verbosity_level)
{
	MRPT_LOG_DEBUG("CMetricMapBuilder ctor.");
}

CMetricMapBuilder::~CMetricMapBuilder()
{
	MRPT_LOG_DEBUG("CMetricMapBuilder dtor.");
}

/*---------------------------------------------------------------
					clear
Clear all elements of the maps, and reset localization to (0,0,0deg).
  ---------------------------------------------------------------*/
void CMetricMapBuilder::clear()
{
	MRPT_LOG_DEBUG("CMetricMapBuilder::clear() called.");
	CSimpleMap dummyMap;
	CPosePDFGaussian dummyPose;

	// Initialize with an empty map and pose to (0,0,0)
	initialize(dummyMap, &dummyPose);
}

/*---------------------------------------------------------------
					loadCurrentMapFromFile
  ---------------------------------------------------------------*/
void CMetricMapBuilder::loadCurrentMapFromFile(const std::string& fileName)
{
	CSimpleMap map;

	// New file??
	if (mrpt::system::fileExists(fileName))
	{
		MRPT_LOG_INFO_STREAM(
			"[CMetricMapBuilder::loadCurrentMapFromFile] Loading current map "
			"from '"
			<< fileName << "' ..." << std::endl);
		CFileGZInputStream f(fileName);

		// Load from file:
		archiveFrom(f) >> map;
	}
	else
	{  // Is a new file, start with an empty map:
		MRPT_LOG_WARN_STREAM(
			"[CMetricMapBuilder::loadCurrentMapFromFile] Loading current map "
			"from '"
			<< fileName << "' ..." << std::endl);
		map.clear();
	}

	// Initialize the map builder with this map
	initialize(map);
}

/*---------------------------------------------------------------
					saveCurrentMapToFile
  ---------------------------------------------------------------*/
void CMetricMapBuilder::saveCurrentMapToFile(
	const std::string& fileName, bool compressGZ) const
{
	// get current map:
	CSimpleMap curmap;
	getCurrentlyBuiltMap(curmap);

	MRPT_LOG_INFO_STREAM(
		"[CMetricMapBuilder::saveCurrentMapToFile] Saving current map to '"
		<< fileName << "' ..." << std::endl);

	// Save to file:
	if (compressGZ)
	{
		CFileGZOutputStream f(fileName);
		archiveFrom(f) << curmap;
	}
	else
	{
		CFileOutputStream f(fileName);
		archiveFrom(f) << curmap;
	}
}
