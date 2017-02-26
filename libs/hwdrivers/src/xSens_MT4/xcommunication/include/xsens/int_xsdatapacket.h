/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef INT_XSDATAPACKET_H
#define INT_XSDATAPACKET_H

struct XsDataPacket;
struct LegacyDataPacket;

#ifdef __cplusplus
extern "C" 
{
#endif

void XsDataPacket_assignFromXsLegacyDataPacket(struct XsDataPacket* thisPtr, struct LegacyDataPacket const* pack, int index);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // file guard
