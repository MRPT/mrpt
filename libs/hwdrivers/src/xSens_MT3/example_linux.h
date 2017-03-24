/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef __EXAMPLE_LINUX_H
#define __EXAMPLE_LINUX_H

int doHardwareScan(xsens::Cmt3 &, CmtDeviceId []);
void doMtSettings(xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId []);
void getUserInputs(CmtOutputMode &, CmtOutputSettings &);
void writeHeaders(unsigned long, CmtOutputMode &, CmtOutputSettings &, int&, int&);
int calcScreenOffset(CmtOutputMode &, CmtOutputSettings &, int);

#endif
