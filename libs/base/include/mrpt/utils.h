/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_utils_H
#define mrpt_utils_H

#include <mrpt/utils/utils_defs.h>

#include <mrpt/poses.h>  // Dependency

#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CStringList.h>
#include <mrpt/utils/TEnumType.h>

// Smart pointers and RTTI:
#include <mrpt/utils/CObject.h>

// CStream related classes:
#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/CMemoryChunk.h>
#include <mrpt/utils/CStdOutStream.h>
#include <mrpt/utils/CFileStream.h>

#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>

// TCP sockets:
#include <mrpt/utils/CServerTCPSocket.h>
#include <mrpt/utils/CClientTCPSocket.h>

#include <mrpt/utils/CEnhancedMetaFile.h>
#include <mrpt/utils/CCanvas.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CMappedImage.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/CSimpleDatabase.h>
#include <mrpt/utils/CPropertiesValuesList.h>
#include <mrpt/utils/CMHPropertiesValuesList.h>
#include <mrpt/utils/CTypeSelector.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CMessage.h>

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CThreadSafeQueue.h>
#include <mrpt/utils/CMessageQueue.h>
#include <mrpt/utils/CDynamicGrid.h>
#include <mrpt/utils/CProbabilityDensityFunction.h>

#include <mrpt/utils/CConsoleRedirector.h>
//#include <mrpt/utils/stl_extensions.h>
//#include <mrpt/utils/metaprogramming.h>
#include <mrpt/utils/exceptions.h>
#include <mrpt/utils/crc.h>
#include <mrpt/utils/md5.h>
#include <mrpt/utils/net_utils.h>
#include <mrpt/utils/CLog.h>
#include <mrpt/utils/CListOfClasses.h>
#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/utils/CRateTimer.h>

#include <mrpt/utils/TCamera.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/utils/PLY_import_export.h>

// Observer-Observable pattern:
#include <mrpt/utils/CObservable.h>
#include <mrpt/utils/CObserver.h>
#include <mrpt/utils/mrptEvent.h>

// Adapter patterns:
#include <mrpt/utils/adapters.h>

#endif

