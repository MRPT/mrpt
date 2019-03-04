/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#ifndef INT_XSDATAPACKET_H
#define INT_XSDATAPACKET_H

struct XsDataPacket;
struct LegacyDataPacket;

#ifdef __cplusplus
extern "C"
{
#endif

	void XsDataPacket_assignFromXsLegacyDataPacket(
		struct XsDataPacket* thisPtr, struct LegacyDataPacket const* pack,
		int index);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // file guard
