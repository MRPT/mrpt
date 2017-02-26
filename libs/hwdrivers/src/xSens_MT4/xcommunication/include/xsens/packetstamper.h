/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef PACKETSTAMPER_H
#define PACKETSTAMPER_H

#include <xsens/pstdint.h>

struct XsDataPacket;

class PacketStamper
{
public:
	static const int64_t MTSCBOUNDARY;
	static const int64_t MTSCBOUNDARY_LOWMASK;
	static const int64_t MTSCBOUNDARY_HIGHMASK;
	static const int64_t MTSCBOUNDARY_HALF;

	static const int64_t SC8BOUNDARY;
	static const int64_t SC8BOUNDARY_LOWMASK;
	static const int64_t SC8BOUNDARY_HIGHMASK;
	static const int64_t SC8BOUNDARY_HALF;

	static int64_t calculateLargePacketCounter8(int64_t frameCounter, int64_t lastCounter);
	static int64_t calculateLargePacketCounter(int64_t frameCounter, int64_t lastCounter);
	static int64_t calculateLargeSampleTime(int64_t frameTime, int64_t lastTime);
	static int64_t stampPacket(XsDataPacket& pack, XsDataPacket& highest);
};

#endif // file guard
