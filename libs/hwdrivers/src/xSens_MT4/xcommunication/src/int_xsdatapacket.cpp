/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <xsens/xstypesconfig.h>
#include "int_xsdatapacket.h"
#include "legacydatapacket.h"
#include "packetfixeddata.h"
#include "mtwsdidata.h"
#include <xsens/xsbusid.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xsmessage.h>
#include <xsens/xsgpspvtdata.h>

//lint -esym(1512, XsDataPacket)
/*! \internal
	\brief Hack class to access internals of XsDataPacket in C++
*/
class InternalDataPacket : public XsDataPacket {
//lint --e(1511, 1509, 1536)
public:
	//! \brief \copybrief XsDataPacket::m_msg
	inline XsMessage& msg()
	{
		return m_msg;
	}

	//! \brief \copybrief XsDataPacket::m_legacyMsg
	inline XsMessage& legacyMsg()
	{
		return m_legacyMsg;
	}

	//! \brief \copybrief XsDataPacket::m_deviceId
	inline XsDeviceId& deviceId()
	{
		return m_deviceId;
	}

	//! \brief \copybrief XsDataPacket::m_lastFoundId
	inline XsDataIdentifier& lastFoundId()
	{
		return m_lastFoundId;
	}

	//! \brief \copybrief XsDataPacket::m_lastFoundOffset
	inline int& lastFoundOffset()
	{
		return m_lastFoundOffset;
	}

	//! \brief \copybrief XsDataPacket::m_itemCount
	inline uint16_t& itemCount()
	{
		return m_itemCount;
	}

	//! \brief \copybrief XsDataPacket::m_originalMessageLength
	inline uint16_t& originalMessageLength()
	{
		return m_originalMessageLength;
	}

	//! \brief \copybrief XsDataPacket::m_toa
	inline XsTimeStamp& toa()
	{
		return m_toa;
	}

	//! \brief \copybrief XsDataPacket::m_packetId
	inline XsTimeStamp& packetId()
	{
		return m_packetId;
	}
};

/*! \class XsDataPacket
	\brief Contains data received from a device or read from a file

	This class is used by XDA for easy access to data contained in a message. It analyzes its internal
	XsMessage upon contruction to give access to the individual contained pieces of data.
	New data can also be added to the %XsDataPacket or updated if it already existed.
*/
extern "C" {

void validatePacket(XsDataPacket* thisPtr);

void XsDataPacket_assignFromXsLegacyDataPacket(struct XsDataPacket* thisPtr, struct LegacyDataPacket const* pack, int index)
{
	assert(pack);

	InternalDataPacket* hacket = (InternalDataPacket*) thisPtr;

	hacket->legacyMsg() = pack->message();

	if (!pack->isXbusSystem())
		index = 0;

	hacket->message().clear();
	if (pack->isXbusSystem())
		hacket->message().setBusId(index + 1);
	else
		hacket->message().setBusId(XS_BID_MASTER);

	hacket->message().setMessageId(XMID_MtData2);
	//hacket->m_legacyMsg = pack->message();
	hacket->deviceId() = pack->deviceId(index);
	hacket->itemCount() = 1;
	hacket->toa() = pack->timeOfArrival();
	hacket->originalMessageLength() = (uint16_t)pack->originalMessage().getDataSize();
	hacket->packetId() = pack->largePacketCounter();
	PacketInfo info = pack->packetInfo(index);
	XsDataFormat format = pack->dataFormat(index);
	XsDataIdentifier di;
	switch (format.m_outputSettings & XOS_Dataformat_Mask) {
	case XOS_Dataformat_Float:
		di = XDI_SubFormatFloat;
		break;
	case XOS_Dataformat_Double:
		di = XDI_SubFormatDouble;
		break;
	case XOS_Dataformat_Fp1632:
		di = XDI_SubFormatFp1632;
		break;
	case XOS_Dataformat_F1220:
		di = XDI_SubFormatFp1220;
		break;
	default:
		di = XDI_None;
		break;
	}
	if (pack->containsRawData(index))
		hacket->setRawData(pack->rawData(index));
	if (pack->rawTemperatureChannelCount(index) == 4)
	{
		XsUShortVector rawGyroTemperatures;
		rawGyroTemperatures[0] = pack->rawTemperature(index, 1);
		rawGyroTemperatures[1] = pack->rawTemperature(index, 2);
		rawGyroTemperatures[2] = pack->rawTemperature(index, 3);
		hacket->setRawGyroscopeTemperatureData(rawGyroTemperatures);
	}
	if (pack->containsCalibratedAcceleration(index))
		hacket->setCalibratedAcceleration(pack->calibratedAcceleration(index));
	if (pack->containsCalibratedGyroscopeData(index))
	{
		// Special copy to preserve watermarking
		hacket->message().setDataShort(XDI_RateOfTurn | di, hacket->message().getDataSize());
		hacket->message().setDataByte(3*hacket->getFPValueSize(di), hacket->message().getDataSize());
		hacket->itemCount()++;
		hacket->message().setDataBuffer(hacket->legacyMsg().getDataBuffer(info.m_calGyr), 3*hacket->getFPValueSize(di), XsDataPacket_itemOffsetExact(thisPtr, XDI_RateOfTurn | di));
	}
	if (pack->containsCalibratedMagneticField(index))
		hacket->setCalibratedMagneticField(pack->calibratedMagneticField(index));
	if (pack->containsOrientationQuaternion(index))
		hacket->setOrientationQuaternion(pack->orientationQuaternion(index), (format.m_outputSettings & XOS_Coordinates_Ned) ? XDI_CoordSysNed : XDI_CoordSysNwu);
	if (pack->containsOrientationEuler(index))
		hacket->setOrientationEuler(pack->orientationEuler(index), (format.m_outputSettings & XOS_Coordinates_Ned) ? XDI_CoordSysNed : XDI_CoordSysNwu);
	if (pack->containsOrientationMatrix(index))
		hacket->setOrientationMatrix(pack->orientationMatrix(index), (format.m_outputSettings & XOS_Coordinates_Ned) ? XDI_CoordSysNed : XDI_CoordSysNwu);
	if (pack->containsPositionLLA(index))
		hacket->setPositionLLA(pack->positionLLA(index));
	if (pack->containsVelocity(index))
		hacket->setVelocity(pack->velocity(index), (format.m_outputSettings & XOS_Coordinates_Ned) ? XDI_CoordSysNed : XDI_CoordSysNwu);
	if (pack->containsStatus(index))
	{
		bool isDetailed = true;
		uint32_t status = pack->status(index, &isDetailed);

		// For MTw's only, the status needs to be swapEndian32-ed
		if (hacket->deviceId().isMtw())
		{
			status = swapEndian32(status);
		}

		if (isDetailed)
		{
			hacket->setStatus(status);
		}
		else
		{
			hacket->setStatusByte((uint8_t) status);
		}
	}
	if (pack->containsGpsPvtData())
	{
		XsPressure tmp;
		tmp.m_pressure = pack->gpsPvtData(index).m_pressure * 2.0; // Convert to Pascal
		tmp.m_pressureAge = pack->gpsPvtData(index).m_pressureAge;
		hacket->setPressure(tmp);
		XsGpsPvtData pvt;
		pvt = pack->gpsPvtData(index);
		hacket->setGpsPvtData(pvt);
	}
	if (pack->containsUtcTime(index))
	{
		XsUtcTime time = pack->utcTime(index);
		hacket->setUtcTime(time);
	}
	if (pack->containsMtwSdiData(index))
	{
		// not yet converted:
		//uint8_t			m_timeSync;
		//XsVector3		m_currentBias;

		MtwSdiData mtwsdi = pack->mtwSdiData(index);
		hacket->deviceId() = mtwsdi.m_deviceId;
		{
			// Special copy to preserve watermarking
			hacket->message().setDataShort(XDI_DeltaQ | di, hacket->message().getDataSize());
			hacket->message().setDataByte(4*hacket->getFPValueSize(di), hacket->message().getDataSize());
			hacket->itemCount()++;
			hacket->message().setDataBuffer(hacket->legacyMsg().getDataBuffer(info.m_wOrientationIncrement), 4*hacket->getFPValueSize(di), XsDataPacket_itemOffsetExact(thisPtr, XDI_DeltaQ | di));

			hacket->message().setDataShort(XDI_DeltaV | di, hacket->message().getDataSize());
			hacket->message().setDataByte(3*hacket->getFPValueSize(di), hacket->message().getDataSize());
			hacket->itemCount()++;
			hacket->message().setDataBuffer(hacket->legacyMsg().getDataBuffer(info.m_wVelocityIncrement), 3*hacket->getFPValueSize(di), XsDataPacket_itemOffsetExact(thisPtr, XDI_DeltaV | di));
		}

		//if (mtwsdi.m_aidingData)
		hacket->setCalibratedMagneticField(mtwsdi.m_magnetoMeter);
		if (mtwsdi.m_barometer)
		{
			XsPressure tmp;
			tmp.m_pressure = mtwsdi.m_barometer * 100.0; // convert from millibar to pascal
			tmp.m_pressureAge = mtwsdi.m_barometer?0:255;
			hacket->setPressure(tmp);
		}
		hacket->setFrameRange(XsRange((int) mtwsdi.m_firstFrameNumber, (int) mtwsdi.m_lastFrameNumber));
		hacket->setRssi(mtwsdi.m_rssi);
	}
	if (pack->containsTemperature(index))
		hacket->setTemperature(pack->temperature(index));
	if (pack->containsPacketCounter(index))
		hacket->setPacketCounter(pack->packetCounter(index));

	// enable this when you experience weird crashes in XsByteArray_destruct or XsDataPacket_destruct
	//validatePacket(thisPtr);
}

} // extern "C"
