/*! \file
	\brief	Contains the ScanPorts interface

	This file contains the interfaces for the functions that are required for scanning ports
	for Xsens devices.

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.

	\section FileChangelog	Changelog
	\par 2006-06-09, v0.0.1
	\li Job Mulder:	Created ScanPorts.h
	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/

#ifndef _SCANPORTS_H_2006_06_09
#define _SCANPORTS_H_2006_06_09

#ifndef _CMT_MONOLITHIC
#	include "xsens_list.hpp"
#endif

namespace xsens {

/*! \brief Scan COM ports for connected Xsens devices.

	The cmtScanPorts function will scan registered Xsens USB converters and serial COM ports
	for connected Xsens devices. If the baudrate parameter is 0 (default), it will try to
	connect at all supported baud rates, starting with the most common 115k2, 460k8 and 
	58k6. If the baudrate parameter is non-zero, only the specified baudrate is tried.
	Any detected devices are returned in the ports list, which is sorted by port nr.
	
*/
bool cmtScanPorts(List<CmtPortInfo>& ports, uint32_t baudrate = 0, uint32_t singleScanTimeout = 1000, uint32_t scanTries = 1);

/*! \brief Scan a single COM port for connected Xsens devices.

	The cmtScanPort function will scan a single port for connected Xsens devices. If the
	baudrate parameter is 0 (default), it will try to connect at all supported baud 
	rates, starting with the most common 115k2, 460k8 and 58k6. If the baudrate parameter
	is non-zero, only the specified baud rate is tried. Any detected devices are returned
	in the portInfo parameter.
	
*/
bool cmtScanPort(CmtPortInfo& portInfo, uint32_t baudrate = 0, uint32_t singleScanTimeout = 1000, uint32_t scanTries = 1);

//! Set to true from another thread to abort any scan currently in progress.
extern bool abortScan;

} // end of xsens namespace

#endif	// _SCANPORTS_H_2006_06_09
