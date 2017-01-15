/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <xsens/xsdatapacket.h>
#include "packetstamper.h"

/*! \class PacketStamper
	\brief Supplies functionality for timestamping data packets.
	\details This class can analyze a data packet and create a proper packet id for it.
*/

//! \brief 16 bit MT Sample Counter boundary
const int64_t PacketStamper::MTSCBOUNDARY = 0x00010000LL;
//! \brief 16 bit MT Sample Counter boundary inclusive mask
const int64_t PacketStamper::MTSCBOUNDARY_LOWMASK	= (MTSCBOUNDARY-1);
//! \brief 16 bit MT Sample Counter boundary exclusive mask
const int64_t PacketStamper::MTSCBOUNDARY_HIGHMASK = (~MTSCBOUNDARY_LOWMASK);
//! \brief 16 bit MT Sample Counter boundary/2, used for determining SC wrapping
const int64_t PacketStamper::MTSCBOUNDARY_HALF = (MTSCBOUNDARY/2);

//! \brief 8 bit Sample Counter boundary
const int64_t PacketStamper::SC8BOUNDARY = 0x00000100LL;
//! \brief 8 bit Sample Counter boundary inclusive mask
const int64_t PacketStamper::SC8BOUNDARY_LOWMASK	= (SC8BOUNDARY-1);
//! \brief 8 bit Sample Counter boundary exclusive mask
const int64_t PacketStamper::SC8BOUNDARY_HIGHMASK = (~SC8BOUNDARY_LOWMASK);
//! \brief 8 bit Sample Counter boundary/2, used for determining SC wrapping
const int64_t PacketStamper::SC8BOUNDARY_HALF = (SC8BOUNDARY/2);

/*! \brief Create 64 bit counter for a packet.
	\details Wrap when new XsDataPacket is too far away from the previous XsDataPacket in time.
	Use half cache size as reasonable time difference
	When infinite cache, simply wrap when new is lower than old
	\param pack The XsDataPacket that needs its 64-bit sample counter updated
	\param highestPacket The highest packet available for the current device, it will be updated if
		the new counter is higher than the stored value.
	\returns The computed counter for the packet.
*/
int64_t PacketStamper::stampPacket(XsDataPacket& pack, XsDataPacket& highestPacket)
{
//	int did = 0;
//	if (pack.containsMtwSdiData())
//	{
//		MtwSdiData sdi = pack.mtwSdiData();
//		did = sdi.m_deviceId;
//		JLDEBUG(gJournal, "XsensDeviceAPI", "%s [%08x] SDI interval (%d-%d)\n", __FUNCTION__, sdi.m_deviceId, sdi.m_firstFrameNumber, sdi.m_lastFrameNumber);
//	}

	//! \todo This could be a (couple of) milliseconds too late, this should be set as soon as the source message arrives: mantis 7157
	pack.setTimeOfArrival(XsTimeStamp::now());
	int64_t newCounter, lastCounter = -1;

	if (!highestPacket.empty())
		lastCounter = highestPacket.packetId().msTime();

	if (pack.containsPacketCounter())
		newCounter = calculateLargePacketCounter(pack.packetCounter(), lastCounter);
	else if (pack.containsSampleTimeFine())
	{
		if (pack.containsSampleTimeCoarse())
			newCounter = (int64_t) pack.sampleTime64();
		else
			newCounter = calculateLargeSampleTime((int32_t) pack.sampleTimeFine(), lastCounter);
	}
	else if (pack.containsPacketCounter8())
		newCounter = calculateLargePacketCounter8(pack.packetCounter8(), lastCounter);
	else
		newCounter = lastCounter + 1;

//	JLDEBUG(gJournal, "XsensDeviceAPI", "%s [%08x] old = %I64d new = %I64d diff = %I64d\n", __FUNCTION__, did, lastCounter, newCounter, (newCounter-lastCounter));

	pack.setPacketId(newCounter);
	if (newCounter > lastCounter)
		highestPacket = pack;

	return newCounter;
}

/*! \brief Calculate the new large packet counter value based on \a frameCounter and the \a lastCounter
	\details Wraparound is at the 8-bit boundary
	\param[in] frameCounter The frame counter
	\param[in] lastCounter The last counter
	\returns The computed packet counter value
	\note If lastCounter < 0, returns frameCounter
*/
int64_t PacketStamper::calculateLargePacketCounter8(int64_t frameCounter, int64_t lastCounter)
{
	if (lastCounter < 0) {
		return frameCounter;
	}

	int64_t low = lastCounter & SC8BOUNDARY_LOWMASK;
	int64_t dt = frameCounter - low;
	if (dt < -SC8BOUNDARY_HALF)
		return lastCounter + dt + SC8BOUNDARY;	// positive wraparound
	if (dt < SC8BOUNDARY_HALF)
		return lastCounter + dt;				// normal increment

	return lastCounter + dt - SC8BOUNDARY;		// negative wraparound
}

/*! \brief Calculate the new large sample counter value based on \a frameCounter and the \a lastCounter
	\details Wraparound is at the 16-bit boundary
	\param[in] frameCounter The frame counter
	\param[in] lastCounter The last counter
	\returns The computed packet counter value
	\note If lastCounter < 0, returns frameCounter
*/
int64_t PacketStamper::calculateLargePacketCounter(int64_t frameCounter, int64_t lastCounter)
{
	if (lastCounter < 0) {
		return frameCounter;
	}

	int64_t low = lastCounter & MTSCBOUNDARY_LOWMASK;
	int64_t dt = frameCounter - low;
	if (dt < -MTSCBOUNDARY_HALF)
		return lastCounter + dt + MTSCBOUNDARY;	// positive wraparound
	if (dt < MTSCBOUNDARY_HALF)
		return lastCounter + dt;				// normal increment

	return lastCounter + dt - MTSCBOUNDARY;		// negative wraparound
}

/*! \brief Calculate the new large sample time value based on \a frameTime and the \a lastTime
	\details Wraparound is at 864000000 (1 day @ 10kHz)
	\param[in] frameTime The frame time
	\param[in] lastTime The last time
	\returns The computed packet counter value
	\note If lastTime < 0, returns frameTime
*/
int64_t PacketStamper::calculateLargeSampleTime(int64_t frameTime, int64_t lastTime)
{
	if (lastTime < 0) {
		return frameTime;
	}
	int64_t low = lastTime % 864000000;
	int64_t dt = frameTime - low;
	if (dt < (-864000000/2))
		return lastTime + dt + 864000000;	// positive wraparound
	if (dt < (864000000/2))
		return lastTime + dt;				// normal increment

	return lastTime + dt - 864000000;		// negative wraparound
}
