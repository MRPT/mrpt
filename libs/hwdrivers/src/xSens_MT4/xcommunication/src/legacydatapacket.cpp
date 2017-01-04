/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include <xsens/xstypesconfig.h>
#include "legacydatapacket.h"
#include "packetfixeddata.h"
#include "mtwsdidata.h"
#include <xsens/xsgpspvtdata.h>
#include <xsens/xspressure.h>
#include <xsens/xscalibrateddata.h>
#include <xsens/xseuler.h>
#include <xsens/xsmatrix3x3.h>
#include <xsens/xsvector3.h>
#include <xsens/xsanalogindata.h>
#include <xsens/xsutctime.h>

#ifdef LOG_PACKET
#	include "xslog.h"
#	define PACKETLOG	XSENSLOG
#else
#	define PACKETLOG(...)	(void)0
#endif

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Packet class //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
#define FORMAT_DOUBLE		((m_fixedData->m_formatList[index].m_outputSettings & XOS_Dataformat_Mask)|XOS_Dataformat_Double)
#define ISDOUBLE(a)			(m_fixedData->m_infoList[index].a >= m_fixedData->m_infoList[index].m_doubleBoundary)
#define CHECKIFDOUBLE(a)	ISDOUBLE(a)?FORMAT_DOUBLE:m_fixedData->m_formatList[index].m_outputSettings

/*!	\struct LegacyDataPacket
	\brief Contains an MTData XsMessage and supports functions for extracting contained data.
*/

/*! \brief Construct a new %LegacyDataPacket without data. The %LegacyDataPacket will be invalid. */
LegacyDataPacket::LegacyDataPacket()
	: m_fixedData(new PacketFixedData)
	, m_lastFoundId(0)
	, m_lastFoundIndex(0)
	, m_rtc(0)
	, m_toa(0)
	, m_packetId(0)
	, m_triggerIn1(0)
	, m_triggerIn2(0)
{
	PACKETLOG("%s creating default %p\n", __FUNCTION__, this);
}

/*! \brief Construct a new %LegacyDataPacket with room for data from \a count devices.
	\param count The number of devices whose data is in the object
	\param xbus Whether the message was sent by an Xbus Master or by a standalone MT. This matters
				because the sample counter is in a different place.
*/
LegacyDataPacket::LegacyDataPacket(uint16_t count, bool xbus)
	: m_fixedData(new PacketFixedData(count))
	, m_lastFoundId(0)
	, m_lastFoundIndex(0)
	, m_rtc(0)
	, m_toa(0)
	, m_packetId(0)
	, m_triggerIn1(0)
	, m_triggerIn2(0)

{
	PACKETLOG("%s creating %p with %d items and xbus option %d\n", __FUNCTION__, this, count, xbus);
	m_fixedData->m_xm = xbus;
}

/*! \brief Construct a LegacyDataPacket as a copy of \a other. */
LegacyDataPacket::LegacyDataPacket(const LegacyDataPacket& other)
	: m_fixedData(0)
	, m_lastFoundId(0)
	, m_lastFoundIndex(0)
	, m_rtc(0)
	, m_toa(0)
	, m_packetId(0)
	, m_triggerIn1(0)
	, m_triggerIn2(0)

{
	PACKETLOG("%s creating %p from %p\n", __FUNCTION__, this, &other);
	*this = other;
	PACKETLOG("%s creating %p from %p done\n", __FUNCTION__, this, &other);
}

/*! \brief Assignment operator, copies the contents of \a pack into this
*/
const LegacyDataPacket& LegacyDataPacket::operator = (const LegacyDataPacket& pack)
{
	PACKETLOG("%s copying from %p to %p\n", __FUNCTION__, &pack, this);
	if (this == &pack)
		return *this;

	delete m_fixedData;
	m_fixedData = NULL;

	if (pack.m_fixedData)	// Can be empty
	{
		m_fixedData = new PacketFixedData(*pack.m_fixedData);
		m_lastFoundId = pack.m_lastFoundId;
		m_lastFoundIndex = pack.m_lastFoundIndex;
	}

	m_toa = pack.m_toa;
	m_rtc = pack.m_rtc;
	m_msg = pack.m_msg;
	m_packetId = pack.m_packetId;
	m_triggerIn1 = pack.m_triggerIn1;
	m_triggerIn2 = pack.m_triggerIn2;

	PACKETLOG("%s copying from %p to %p done\n", __FUNCTION__, &pack, this);
	return *this;
}

/*! \brief Destructor */
LegacyDataPacket::~LegacyDataPacket()
{
	try {
		PACKETLOG("%s destroying %p\n", __FUNCTION__, this);
		delete m_fixedData;
		m_fixedData = NULL;
		PACKETLOG("%s destroyed %p\n", __FUNCTION__, this);
	}
	catch (...)
	{}
}

//lint -esym(613, LegacyDataPacket::m_fixedData) assert and dataSize take care of this
/*! \brief Returns the number of devices whose data is contained in the object
*/
uint16_t LegacyDataPacket::itemCount(void) const
{
	assert(m_fixedData);
	return m_fixedData->m_itemCount;
}

/*! \brief Set the number of devices whose data is contained in this object to \a count
*/
void LegacyDataPacket::setItemCount(uint16_t count)
{
	assert(m_fixedData);
	m_fixedData->m_itemCount = count;
}

/*! \brief Returns the Time Of Arrival value as stored in the object
*/
XsTimeStamp LegacyDataPacket::timeOfArrival(void) const
{
	return m_toa;
}

/*! \brief Set the Time Of Arrival value to \a timeofarrival
*/
void LegacyDataPacket::setTimeOfArrival(XsTimeStamp timeofarrival)
{
	m_toa = timeofarrival;
}

/*! \brief Returns the Real Time Clock value as stored in the object
*/
XsTimeStamp LegacyDataPacket::rtc(void) const
{
	return m_rtc;
}

/*! \brief Set the Real Time Clock value to \a realtimeclock
*/
void LegacyDataPacket::setRtc(const XsTimeStamp realtimeclock)
{
	m_rtc = realtimeclock;
}

/*! \brief Return the 64-bit sample counter associated with this packet
	\returns The 64-bit sample counter associated with this packet
	\note This sample counter may differ a lot from the normal 16-bit sample counter
*/
int64_t LegacyDataPacket::largePacketCounter(void) const
{
	return m_packetId.msTime();
}

/*! \brief Set the 64-bit sample counter associated with this packet
*/
void LegacyDataPacket::setLargePacketCounter(const int64_t sc)
{
	m_packetId = sc;
}

/*! \brief Returns a copy of the %XsMessage contained by the object, including computed and added data. */
XsMessage LegacyDataPacket::message(void) const
{
	return m_msg;
}

/*! \brief Returns the original message as it was received, without computed and added data (except for SDI interval reconstruction)
*/
XsMessage LegacyDataPacket::originalMessage(void) const
{
	assert(m_fixedData);
	uint16_t originalSize = 0;
	for (uint16_t i = 0;i < m_fixedData->m_itemCount;++i)
		if (m_fixedData->m_infoList[i].m_doubleBoundary > originalSize)
			originalSize = m_fixedData->m_infoList[i].m_doubleBoundary;

	if (originalSize == m_msg.getDataSize())
		return m_msg;

	XsMessage m = m_msg;
	m.resizeData(originalSize);
	m.setDataBuffer(m_msg.getDataBuffer(), originalSize);
	m.recomputeChecksum();
	return m;
}

/*! \brief Set the source message to \a msg
	\param msg The message to use
	\note Call updateInfoList() to actually start using the new message
*/
void LegacyDataPacket::setMessage(const XsMessage& msg)
{
	m_msg = msg;
}

/*! \brief Returns the packet info for the \a index'th device in the packet.
	\details This describes what data is contained by the object.
	\param index The index of the device whose packet info should be returned
	\returns The packet info for device \a index
*/
PacketInfo LegacyDataPacket::packetInfo(int32_t index) const
{
	assert(m_fixedData);
	if (index < 0 || index >= m_fixedData->m_itemCount)
		return PacketInfo();

	return m_fixedData->m_infoList[index];
}

/*! \brief Return the frame counter (previously: sample counter) of the packet
	\details For SDI data, this function will return the Last Frame Number in the SDI interval.
	For other data, this function will return the plain Sample Counter.
	\returns The frame counter of the packet or 0 if it isn't present in the packet (note that 0 is
	a valid sample counter, so don't use it for an error check).
*/
uint16_t LegacyDataPacket::frameCounter() const
{
	assert(m_fixedData);
	if (containsPacketCounter(0))
		return m_msg.getDataShort(m_fixedData->m_infoList[0].m_sc);
	else if (containsMtwSdiData(0))
		return m_msg.getDataShort(m_fixedData->m_infoList[0].m_wLastFrameNumber);
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Returns the index of the fixed data with id \a dev
	\details If the result is not -1, it can be used as the \a index parameter in other functions
	that require it (see the list below).
	\param dev The device ID to find
	\returns The index of the fixed data
	\sa deviceId \sa setDeviceId \sa dataFormat \sa setDataFormat(const XsDataFormat&, int32_t)
	\sa setDataFormat(XsOutputMode, XsOutputSettings, int32_t)
*/
int32_t LegacyDataPacket::findDeviceId(XsDeviceId dev) const
{
	assert(m_fixedData);
	if (!dev.isValid())
		return 0;
	if (dev == m_lastFoundId)
		return m_lastFoundIndex;
	for (uint16_t i=0;i<m_fixedData->m_itemCount;++i)
	{
		if (m_fixedData->m_idList[i] == dev)
		{
			m_lastFoundId = dev;
			m_lastFoundIndex = i;
			return i;
		}
	}
	return -1;
}

/*! \brief Returns the device ID of the device with the given \a index
	\param index The index of the device whose device ID should be returned
	\returns The device ID of the device
	\sa findDeviceId
*/
XsDeviceId LegacyDataPacket::deviceId(int32_t index) const
{
	assert(m_fixedData);
	return m_fixedData->m_idList[index];
}

/*! \brief Sets the device ID of the device with the given \a index to \a deviceid
	\param deviceid The device ID to set
	\param index The index of the device whose device ID should be updated
	\returns true if the device ID was successfully updated
	\sa findDeviceId
*/
void LegacyDataPacket::setDeviceId(XsDeviceId deviceid, int32_t index)
{
	assert(m_fixedData);
	m_fixedData->m_idList[index] = deviceid;
}

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Returns the data format of the device with the given \a index
	\param index The index of the device whose data format should be returned
	\returns The data format of the device
	\sa findDeviceId
*/
XsDataFormat LegacyDataPacket::dataFormat(int32_t index) const
{
	if (!m_fixedData || index >= m_fixedData->m_itemCount || m_fixedData->m_formatList == NULL)
		return XsDataFormat();

	return m_fixedData->m_formatList[index];
}

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Sets the data format of the device with the given \a index to \a format
	\param format The data format of the device
	\param index The index of the device whose data format should be updated
	\returns true if the data format was successfully updated
	\sa findDeviceId
*/
bool LegacyDataPacket::setDataFormat(const XsDataFormat& format, int32_t index)
{
	assert(m_fixedData);

	if (index < m_fixedData->m_itemCount)
	{
		m_fixedData->m_formatList[index] = format;
		updateInfoList();
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Sets the data format of the device with the given \a index to \a outputMode and \a outputSettings
	\param outputMode The output mode of the device (see low level communication documentation)
	\param outputSettings The output settings of the device (see low level communication documentation)
	\param index The index of the device whose data format should be updated
	\returns true if the data format was successfully updated
	\sa findDeviceId
*/
bool LegacyDataPacket::setDataFormat(XsOutputMode outputMode, XsOutputSettings outputSettings, int32_t index)
{
	assert(m_fixedData);

	if (index < m_fixedData->m_itemCount)
	{
		m_fixedData->m_formatList[index].m_outputMode = outputMode;
		m_fixedData->m_formatList[index].m_outputSettings = outputSettings;
		updateInfoList();
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Returns whether the xbus flag is set or not
	\returns trur if the xbus flag is set or not
	\sa setXbusSystem
*/
bool LegacyDataPacket::isXbusSystem(void) const
{
	assert(m_fixedData);
	return m_fixedData->m_xm;
}

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Sets the xbus flag
	\details The xbus flag determines where the packet counter is stored in the message. Setting
	this value incorrectly can lead to corrupted data reads.
	\param xbus The desired setting
	\param convert When set to true, the sample counter will be moved from its old place to the new
	place.
*/
void LegacyDataPacket::setXbusSystem(bool xbus, bool convert)
{
	assert(m_fixedData);

	if (xbus != m_fixedData->m_xm)
	{
		if (convert)
		{
			XsMtTimeStamp stamp = packetCounter(0);

			// move the time stamp value(s) around
			if (xbus)
			{
				// new version is Xbus, so old version is regular format -> place timestamp at start of packet and remove from all individual data units
				// remove the timestamp from all data units

				// insert timestamp at the start
				m_msg.insertData(2,0);
				m_msg.setDataShort(stamp,0);
			}
			else
			{
				// new version is regular, so old version is Xbus format -> place timestamp at end of all individual data units and remove from start of packet
				// append the timestamp to all data units

				// remove timestamp from the start
				m_msg.deleteData(2,0);
			}
		}
		// update the state cache
		m_fixedData->m_xm = xbus;
		updateInfoList();
	}
}

/*! \brief Returns the floating/fixed point value size in bytes
	\param index The index of the item whose fp size should be returned.
	\returns The floating/fixed point value size in bytes
*/
uint16_t LegacyDataPacket::getFPValueSize(int32_t index) const
{
	assert(m_fixedData);
	uint16_t ds = 4;
	switch (m_fixedData->m_formatList[index].m_outputSettings & XOS_Dataformat_Mask)
	{
		case XOS_Dataformat_Float:
			ds = 4;
			break;

		case XOS_Dataformat_Double:
			ds = 8;
			break;

		case XOS_Dataformat_Fp1632:
			ds = 6;
			break;

		case XOS_Dataformat_F1220:
			ds = 4;
			break;
		default:
			break;
	}
	return ds;
}

/*! \brief Returns the size of the data
	\param index The index of the item of which the size should be returned.
	\returns The size of the data
*/
XsSize LegacyDataPacket::dataSize(int32_t index) const
{
	if (m_fixedData && index < m_fixedData->m_itemCount)
		return m_fixedData->m_infoList[index].m_size;
	return 0;
}

/*! \brief Update the internal info list by analyzing the known XsDataFormat for each device.
*/
void LegacyDataPacket::updateInfoList()
{
	assert(m_fixedData);
	if (m_fixedData->m_infoList != NULL)
	{
		delete[] m_fixedData->m_infoList;
		m_fixedData->m_infoList = NULL;
	}

	assert(m_fixedData->m_itemCount);
	// allocate list
	m_fixedData->m_infoList = new PacketInfo[m_fixedData->m_itemCount];
	uint16_t totalOffset;
	if (m_fixedData->m_xm)
		totalOffset = 2;
	else
		totalOffset = 0;

	// fill lists
	for (uint16_t i = 0;i < m_fixedData->m_itemCount;++i)
	{
		m_fixedData->m_infoList[i].m_offset = totalOffset;
		m_fixedData->m_infoList[i].m_size = 0;

		uint16_t ds = getFPValueSize(i);

		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Raw)
		{
			m_fixedData->m_infoList[i].m_rawData = totalOffset;
			m_fixedData->m_infoList[i].m_rawAcc = totalOffset;
			m_fixedData->m_infoList[i].m_rawGyr = totalOffset+6;
			m_fixedData->m_infoList[i].m_rawMag = totalOffset+12;
			m_fixedData->m_infoList[i].m_rawTemp[0] = totalOffset+18;

			m_fixedData->m_infoList[i].m_size += 20;
			totalOffset += 20;

			if(m_fixedData->m_formatList[i].m_outputSettings & XOS_ExtendedTemperature_Mask)
			{
				for(int j = 0; j < XS_EXTRA_TEMPERATURE_CHANNELS; j++)
				{
					m_fixedData->m_infoList[i].m_rawTemp[j + 1] = totalOffset;
					m_fixedData->m_infoList[i].m_size += 2;
					totalOffset += 2;
				}
			} else
			{
				for(int j = 0; j < XS_EXTRA_TEMPERATURE_CHANNELS; j++)
				{
					m_fixedData->m_infoList[i].m_rawTemp[j + 1] = XS_DATA_ITEM_NOT_AVAILABLE;
				}
			}

		}
		else
		{
			m_fixedData->m_infoList[i].m_rawData = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_rawAcc = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_rawGyr = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_rawMag = XS_DATA_ITEM_NOT_AVAILABLE;
			for(int j = 0; j < XS_MAX_TEMPERATURE_CHANNELS; j++)
				m_fixedData->m_infoList[i].m_rawTemp[j] = XS_DATA_ITEM_NOT_AVAILABLE;
		}

		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Sdi)
		{
			m_fixedData->m_infoList[i].m_mtwSdiData = totalOffset;
			m_fixedData->m_infoList[i].m_wClientId = totalOffset;
			m_fixedData->m_infoList[i].m_wTimeSync = totalOffset + 4;
			m_fixedData->m_infoList[i].m_wFirstFrameNumber = totalOffset + 5;
			m_fixedData->m_infoList[i].m_wLastFrameNumber = totalOffset + 7;
			m_fixedData->m_infoList[i].m_wCurrentBias = totalOffset + 9;
			m_fixedData->m_infoList[i].m_wOrientationIncrement = totalOffset + 9+3*ds;
			m_fixedData->m_infoList[i].m_wVelocityIncrement = totalOffset + 9+7*ds;
			m_fixedData->m_infoList[i].m_wAidingData = totalOffset + 9+10*ds;
			m_fixedData->m_infoList[i].m_wBaroMeter = totalOffset + 10+10*ds;
			m_fixedData->m_infoList[i].m_wMagnetoMeter = totalOffset + 12+10*ds;
			m_fixedData->m_infoList[i].m_wRssi = totalOffset + 12+13*ds;
			m_fixedData->m_infoList[i].m_size += 13+13*ds;
			totalOffset += 13+13*ds;
		}
		else
		{
			m_fixedData->m_infoList[i].m_mtwSdiData = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wClientId = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wTimeSync = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wFirstFrameNumber = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wLastFrameNumber = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wCurrentBias = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wOrientationIncrement = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wVelocityIncrement = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wAidingData = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wBaroMeter = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wMagnetoMeter = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_wRssi = XS_DATA_ITEM_NOT_AVAILABLE;
		}


		if (m_fixedData->m_formatList[i].m_outputMode & XOM_GpsPvt_Pressure)
		{
			m_fixedData->m_infoList[i].m_gpsPvtData = totalOffset;
			m_fixedData->m_infoList[i].m_gpsPvtPressure = totalOffset;
			m_fixedData->m_infoList[i].m_gpsPvtPressureAge = totalOffset + 2;
			m_fixedData->m_infoList[i].m_size += 3;
			totalOffset += 3;

			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_NoGpsInGpsPvt) == 0)
			{
				m_fixedData->m_infoList[i].m_gpsPvtGpsData = totalOffset;
				m_fixedData->m_infoList[i].m_gpsPvtItow = totalOffset;
				m_fixedData->m_infoList[i].m_gpsPvtLatitude = totalOffset + 4;
				m_fixedData->m_infoList[i].m_gpsPvtLongitude = totalOffset + 8;
				m_fixedData->m_infoList[i].m_gpsPvtHeight = totalOffset + 12;
				m_fixedData->m_infoList[i].m_gpsPvtVeln = totalOffset + 16;
				m_fixedData->m_infoList[i].m_gpsPvtVele = totalOffset + 20;
				m_fixedData->m_infoList[i].m_gpsPvtVeld = totalOffset + 24;
				m_fixedData->m_infoList[i].m_gpsPvtHacc = totalOffset + 28;
				m_fixedData->m_infoList[i].m_gpsPvtVacc = totalOffset + 32;
				m_fixedData->m_infoList[i].m_gpsPvtSacc = totalOffset + 36;
				m_fixedData->m_infoList[i].m_gpsPvtGpsAge = totalOffset + 40;
				m_fixedData->m_infoList[i].m_size += 41;
				totalOffset += 41;
			}
			else
			{
				m_fixedData->m_infoList[i].m_gpsPvtGpsData = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtItow = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtLatitude = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtLongitude = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtHeight = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtVeln = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtVele = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtVeld = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtHacc = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtVacc = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtSacc = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_gpsPvtGpsAge = XS_DATA_ITEM_NOT_AVAILABLE;
			}
		}
		else
		{
			m_fixedData->m_infoList[i].m_gpsPvtData = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtPressure = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtPressureAge = XS_DATA_ITEM_NOT_AVAILABLE;

			m_fixedData->m_infoList[i].m_gpsPvtGpsData = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtItow = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtLatitude = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtLongitude = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtHeight = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtVeln = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtVele = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtVeld = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtHacc = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtVacc = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtSacc = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_gpsPvtGpsAge = XS_DATA_ITEM_NOT_AVAILABLE;
		}

		for(int j = 0; j < XS_MAX_TEMPERATURE_CHANNELS; j++)
			m_fixedData->m_infoList[i].m_temp[j] = XS_DATA_ITEM_NOT_AVAILABLE;

		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Temperature)
		{
			int tCount = (m_fixedData->m_formatList[i].m_outputSettings & XOS_ExtendedTemperature_Mask) ? XS_MAX_TEMPERATURE_CHANNELS : 1;
			for(int j = 0; j < tCount; j++)
			{
				m_fixedData->m_infoList[i].m_temp[j] = totalOffset;
				m_fixedData->m_infoList[i].m_size += ds;
				totalOffset += ds;
			}
		}

		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Calibrated)
		{
			m_fixedData->m_infoList[i].m_calData = totalOffset;
			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_CalibratedMode_Acc_Mask) == 0)
			{
				m_fixedData->m_infoList[i].m_calAcc = totalOffset;
				m_fixedData->m_infoList[i].m_size += 3*ds;
				totalOffset += 3*ds;
			}
			else
				m_fixedData->m_infoList[i].m_calAcc = XS_DATA_ITEM_NOT_AVAILABLE;

			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_CalibratedMode_Gyr_Mask) == 0)
			{
				m_fixedData->m_infoList[i].m_calGyr = totalOffset;
				m_fixedData->m_infoList[i].m_size += 3*ds;
				totalOffset += 3*ds;
			}
			else
				m_fixedData->m_infoList[i].m_calGyr = XS_DATA_ITEM_NOT_AVAILABLE;

			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_CalibratedMode_Mag_Mask) == 0)
			{
				m_fixedData->m_infoList[i].m_calMag = totalOffset;
				m_fixedData->m_infoList[i].m_size += 3*ds;
				totalOffset += 3*ds;
			}
			else
				m_fixedData->m_infoList[i].m_calMag = XS_DATA_ITEM_NOT_AVAILABLE;

			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_CalibratedMode_AccGyrMag_Mask) == XOS_CalibratedMode_AccGyrMag_Mask)
				m_fixedData->m_infoList[i].m_calData = XS_DATA_ITEM_NOT_AVAILABLE;
		}
		else
		{
			m_fixedData->m_infoList[i].m_calData = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_calAcc = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_calGyr = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_calMag = XS_DATA_ITEM_NOT_AVAILABLE;
		}

		m_fixedData->m_infoList[i].m_oriEul = XS_DATA_ITEM_NOT_AVAILABLE;
		m_fixedData->m_infoList[i].m_oriQuat = XS_DATA_ITEM_NOT_AVAILABLE;
		m_fixedData->m_infoList[i].m_oriMat = XS_DATA_ITEM_NOT_AVAILABLE;

		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Orientation)
		{
			switch (m_fixedData->m_formatList[i].m_outputSettings & XOS_OrientationMode_Mask)
			{
			case XOS_OrientationMode_Euler:
				m_fixedData->m_infoList[i].m_oriEul = totalOffset;
				m_fixedData->m_infoList[i].m_size += 3*ds;
				totalOffset += 3*ds;
				break;
			case XOS_OrientationMode_Quaternion:
				m_fixedData->m_infoList[i].m_oriQuat = totalOffset;
				m_fixedData->m_infoList[i].m_size += 4*ds;
				totalOffset += 4*ds;
				break;
			case XOS_OrientationMode_Matrix:
				m_fixedData->m_infoList[i].m_oriMat = totalOffset;
				m_fixedData->m_infoList[i].m_size += 9*ds;
				totalOffset += 9*ds;
				break;
			default:
				break;
			}
		}

		m_fixedData->m_infoList[i].m_analogIn1 = XS_DATA_ITEM_NOT_AVAILABLE;
		m_fixedData->m_infoList[i].m_analogIn2 = XS_DATA_ITEM_NOT_AVAILABLE;
		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Auxiliary)
		{
			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_AuxiliaryMode_Ain1_Mask) == 0)
			{
				m_fixedData->m_infoList[i].m_analogIn1 = totalOffset;
				m_fixedData->m_infoList[i].m_size += 2;
				totalOffset += 2;
			}
			else
				m_fixedData->m_infoList[i].m_analogIn1 = XS_DATA_ITEM_NOT_AVAILABLE;

			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_AuxiliaryMode_Ain2_Mask) == 0)
			{
				m_fixedData->m_infoList[i].m_analogIn2 = totalOffset;
				m_fixedData->m_infoList[i].m_size += 2;
				totalOffset += 2;
			}
			else
				m_fixedData->m_infoList[i].m_analogIn2 = XS_DATA_ITEM_NOT_AVAILABLE;
		}

		m_fixedData->m_infoList[i].m_posLLA = XS_DATA_ITEM_NOT_AVAILABLE;
		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Position)
		{
			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_PositionMode_Mask) == XOS_PositionMode_Lla_Wgs84)
			{
				m_fixedData->m_infoList[i].m_posLLA = totalOffset;
				m_fixedData->m_infoList[i].m_size += 3*ds;
				totalOffset += 3*ds;
			}
		}

		m_fixedData->m_infoList[i].m_velNEDorNWU = XS_DATA_ITEM_NOT_AVAILABLE;
		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Velocity)
		{
			if ((m_fixedData->m_formatList[i].m_outputSettings & XOS_VelocityMode_Mask) == XOS_VelocityMode_Ms_Xyz)
			{
				m_fixedData->m_infoList[i].m_velNEDorNWU = totalOffset;
				m_fixedData->m_infoList[i].m_size += 3*ds;
				totalOffset += 3*ds;
			}
		}

		if (m_fixedData->m_formatList[i].m_outputMode & XOM_Status)
		{
			m_fixedData->m_infoList[i].m_status = totalOffset;

			if (m_fixedData->m_formatList[i].m_outputSettings & XOS_Status_Detailed)
			{
				m_fixedData->m_infoList[i].m_detailedStatus = totalOffset;
				m_fixedData->m_infoList[i].m_size += 4;
				totalOffset += 4;
			}
			else
			{
				m_fixedData->m_infoList[i].m_detailedStatus = XS_DATA_ITEM_NOT_AVAILABLE;
				m_fixedData->m_infoList[i].m_size += 1;
				totalOffset += 1;
			}
		}
		else
		{
			m_fixedData->m_infoList[i].m_status = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_detailedStatus = XS_DATA_ITEM_NOT_AVAILABLE;
		}

		m_fixedData->m_infoList[i].m_sc = XS_DATA_ITEM_NOT_AVAILABLE;
		if (m_fixedData->m_xm)
			m_fixedData->m_infoList[i].m_sc = 0;
		if (m_fixedData->m_formatList[i].m_outputSettings & XOS_Timestamp_PacketCounter)
		{
			if (!m_fixedData->m_xm)
				m_fixedData->m_infoList[i].m_sc = totalOffset;
			m_fixedData->m_infoList[i].m_size += 2;
			totalOffset += 2;
		}

		if (m_fixedData->m_formatList[i].m_outputSettings & XOS_Timestamp_SampleUtc)
		{
			m_fixedData->m_infoList[i].m_utcTime = totalOffset;
			m_fixedData->m_infoList[i].m_utcNano = totalOffset;
			m_fixedData->m_infoList[i].m_utcYear = totalOffset + 4;
			m_fixedData->m_infoList[i].m_utcMonth = totalOffset + 6;
			m_fixedData->m_infoList[i].m_utcDay = totalOffset + 7;
			m_fixedData->m_infoList[i].m_utcHour = totalOffset + 8;
			m_fixedData->m_infoList[i].m_utcMinute = totalOffset + 9;
			m_fixedData->m_infoList[i].m_utcSecond = totalOffset + 10;
			m_fixedData->m_infoList[i].m_utcValid = totalOffset + 11;
			m_fixedData->m_infoList[i].m_size += 12;
			totalOffset += 12;
		}
		else {
			m_fixedData->m_infoList[i].m_utcTime = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_utcNano = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_utcYear = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_utcMonth = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_utcDay = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_utcHour = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_utcMinute = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_utcSecond = XS_DATA_ITEM_NOT_AVAILABLE;
			m_fixedData->m_infoList[i].m_utcValid = XS_DATA_ITEM_NOT_AVAILABLE;
		}

		// post-processing data is never available at this point
		m_fixedData->m_infoList[i].m_acc_g = XS_DATA_ITEM_NOT_AVAILABLE;
		m_fixedData->m_infoList[i].m_doubleBoundary = totalOffset;

		if (m_fixedData->m_infoList[i].m_sc == XS_DATA_ITEM_NOT_AVAILABLE)
			m_fixedData->m_infoList[i].m_sc = m_fixedData->m_infoList[i].m_wLastFrameNumber;
	}
}

#if 0
/*! \brief Take packet fixed data out of this packet
	\details The packet fixed data can usually be shared across several other packets. When
	implementing caching, the fixed packet data can cause a huge overhead. By removing it from the
	packet, the amount of memory usage is significantly reduced.
*/
PacketFixedData LegacyDataPacket::takePacketFixedData()
{
	if (m_fixedData == NULL)
		return PacketFixedData();

	PacketFixedData data(*m_fixedData);
	CHKDELNUL(m_fixedData);
	return data;
}


/*! \brief Put packet fixed data back into this packet
	\see takePacketFixedData
*/
void LegacyDataPacket::putPacketFixedData(const PacketFixedData& data)
{
	CHKDELNUL(m_fixedData);
	m_fixedData = new PacketFixedData(data);
}
#endif

/*! \brief The Raw Accelerometer component of a data item.

	\param index The index of the item of which the data should be returned.

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector LegacyDataPacket::rawAcceleration(int32_t index) const
{
	XsUShortVector buffer;
	if (containsRawAcceleration(index))
		for (uint16_t i=0;i<3;++i)
			buffer[i] = m_msg.getDataShort(m_fixedData->m_infoList[index].m_rawAcc + (2*i));

	return buffer;
}

/*! \brief Check if data item contains Raw Accelerometer data

  \param index The index of the item of which the data should be returned

  \returns true if this packet contains raw acceleration data
*/
bool LegacyDataPacket::containsRawAcceleration(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_rawAcc == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}

/*! \brief Add/update Raw Accelerometer data for the item */
bool LegacyDataPacket::setRawAcceleration(const XsUShortVector& vec, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_rawAcc == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_rawAcc = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_fixedData->m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec[i], m_fixedData->m_infoList[index].m_rawAcc + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Gyroscope component of a data item.
XsUShortVector LegacyDataPacket::rawGyroscopeData(int32_t index) const
{
	XsUShortVector buffer;
	if (containsRawGyroscopeData(index))
		for (uint16_t i=0;i<3;++i)
			buffer[i] = m_msg.getDataShort(m_fixedData->m_infoList[index].m_rawGyr + (2*i));

	return buffer;
}
bool LegacyDataPacket::containsRawGyroscopeData(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_rawGyr == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setRawGyroscopeData(const XsUShortVector& vec, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_rawGyr == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_rawGyr = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_fixedData->m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec[i], m_fixedData->m_infoList[index].m_rawGyr + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Magnetometer component of a data item.
XsUShortVector LegacyDataPacket::rawMagneticField(int32_t index) const
{
	XsUShortVector buffer;
	if (containsRawMagneticField(index))
		for (uint16_t i=0;i<3;++i)
			buffer[i] = m_msg.getDataShort(m_fixedData->m_infoList[index].m_rawMag + (2*i));

	return buffer;
}
bool LegacyDataPacket::containsRawMagneticField(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_rawMag == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setRawMagneticField(const XsUShortVector& vec, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_rawMag == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_rawMag = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_fixedData->m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec[i], m_fixedData->m_infoList[index].m_rawMag + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Temperature component of a data item.
uint16_t LegacyDataPacket::rawTemperature(int32_t index, int channel) const
{
	if (!containsRawTemperature(index, channel))
		return 0;

	return m_msg.getDataShort(m_fixedData->m_infoList[index].m_rawTemp[channel]);
}
bool LegacyDataPacket::containsRawTemperature(int32_t index, int channel) const
{
	if (dataSize(index) == 0)
		return false;
	if (channel >= XS_MAX_TEMPERATURE_CHANNELS)
		return false;
	if (m_fixedData->m_infoList[index].m_rawTemp[channel] == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setRawTemperature(const uint16_t temp, int32_t index, int channel)
{
	if (dataSize(index) == 0)
		return false;
	if (channel >= XS_MAX_TEMPERATURE_CHANNELS)
		return false;
	if (m_fixedData->m_infoList[index].m_rawTemp[channel] == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		int c = rawTemperatureChannelCount(index);
		for(int i = c; i <= channel; i++) //Add (also add missing intermediate channels)
		{
			m_fixedData->m_infoList[index].m_rawTemp[i] = (uint16_t) m_msg.getDataSize();
			m_msg.resizeData(m_msg.getDataSize() + 2);
			m_fixedData->m_infoList[index].m_size += 2;
		}
	}
	// update
	m_msg.setDataShort(temp, m_fixedData->m_infoList[index].m_rawTemp[channel]);
	return true;
}
int LegacyDataPacket::rawTemperatureChannelCount(int32_t index) const
{
	for(int i = 0; i < XS_MAX_TEMPERATURE_CHANNELS; i++)
	{
		if(m_fixedData->m_infoList[index].m_rawTemp[i] == XS_DATA_ITEM_NOT_AVAILABLE)
			return i;
	}
	return XS_MAX_TEMPERATURE_CHANNELS;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Data component of a data item.
XsScrData LegacyDataPacket::rawData(int32_t index) const
{
	XsScrData buffer;
	if (containsRawData(index))
	{
		const uint8_t* tmp = m_msg.getDataBuffer(m_fixedData->m_infoList[index].m_rawData);
		const uint16_t* sh = (const uint16_t*) tmp;
		uint16_t* bare = (uint16_t*) &buffer;

		for (uint16_t i=0;i<(9+rawTemperatureChannelCount(index));++i, ++sh, ++bare)
			*bare = swapEndian16(*sh);// m_msg.getDataShort(m_fixedData->m_infoList[index].m_rawData + (2*i));
	}
	return buffer;
}
bool LegacyDataPacket::containsRawData(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	return (m_fixedData->m_infoList[index].m_rawData != XS_DATA_ITEM_NOT_AVAILABLE);
}
bool LegacyDataPacket::setRawData(const XsScrData& data, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_rawData == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_rawData = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3* 3*2 + 2);
		m_fixedData->m_infoList[index].m_rawAcc = m_fixedData->m_infoList[index].m_rawData;
		m_fixedData->m_infoList[index].m_rawGyr = m_fixedData->m_infoList[index].m_rawData + 3*2;
		m_fixedData->m_infoList[index].m_rawMag = m_fixedData->m_infoList[index].m_rawData + 6*2;
		for(int i = 0; i < XS_MAX_TEMPERATURE_CHANNELS; i++)
		{
			m_fixedData->m_infoList[index].m_rawTemp[i]= m_fixedData->m_infoList[index].m_rawData + 9*2 + (i * 2);
		}
		m_fixedData->m_infoList[index].m_size += 3* 3*2 + (XS_MAX_TEMPERATURE_CHANNELS * 2);
	}
	// update
	int16_t* bare = (int16_t*) &data;
	for (uint16_t i=0;i<(9+rawTemperatureChannelCount(index));++i)
		m_msg.setDataShort(bare[i], m_fixedData->m_infoList[index].m_rawData + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Gps PVT Data component of a data item.
XsGpsPvtData LegacyDataPacket::gpsPvtData(int32_t index) const
{
	XsGpsPvtData buffer;
	if (containsGpsPvtData(index))
	{
		//const uint8_t* tmp = m_msg.getDataBuffer(m_fixedData->m_infoList[index].m_gpsPvtData);
		//const uint16_t* sh = (const uint16_t*) tmp;
		//uint16_t* bare = (uint16_t*) &buffer;

		// pressure data
		buffer.m_pressure = m_msg.getDataShort(m_fixedData->m_infoList[index].m_gpsPvtPressure);
		// pressAge
		buffer.m_pressureAge = m_msg.getDataByte(m_fixedData->m_infoList[index].m_gpsPvtPressureAge);

		// lon,lat,height,hacc,vacc,veln,vele,veld
		//tmp = m_msg.getDataBuffer(m_fixedData->m_infoList[index].m_gpsPvtGpsData);
		//const uint32_t* ln = (const uint32_t*) tmp;
		uint32_t *bareln = (uint32_t*) &buffer.m_itow;
		for (uint16_t i=0; i < 10; ++i)
		{
			//lint --e{662, 661}
			if (m_fixedData->m_infoList[index].m_gpsPvtGpsData != XS_DATA_ITEM_NOT_AVAILABLE)
			{
				bareln[i] = m_msg.getDataLong(m_fixedData->m_infoList[index].m_gpsPvtGpsData + (4*i));	//lint !e661 !e662
			}
			else
			{
				bareln[i] = 0;
			}
		}

		// gpsAge
		if (m_fixedData->m_infoList[index].m_gpsPvtGpsAge != XS_DATA_ITEM_NOT_AVAILABLE)
		{
			buffer.m_gpsAge = m_msg.getDataByte(m_fixedData->m_infoList[index].m_gpsPvtGpsAge);
		}
		else
		{
			buffer.m_gpsAge = 0;
		}
	}
	else
		buffer.clear();
	return buffer;
}
bool LegacyDataPacket::containsGpsPvtData(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_gpsPvtData == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setGpsPvtData(const XsGpsPvtData& data, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_gpsPvtData == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_gpsPvtData = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + (2+1) + (40 + 1));
		m_fixedData->m_infoList[index].m_gpsPvtPressure = m_fixedData->m_infoList[index].m_gpsPvtData;
		m_fixedData->m_infoList[index].m_gpsPvtPressureAge = m_fixedData->m_infoList[index].m_gpsPvtData + 2;

		m_fixedData->m_infoList[index].m_gpsPvtGpsData = m_fixedData->m_infoList[index].m_gpsPvtData + 3;
		m_fixedData->m_infoList[index].m_gpsPvtItow = m_fixedData->m_infoList[index].m_gpsPvtData + 3;
		m_fixedData->m_infoList[index].m_gpsPvtLatitude = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 4;
		m_fixedData->m_infoList[index].m_gpsPvtLongitude = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 8;
		m_fixedData->m_infoList[index].m_gpsPvtHeight = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 12;
		m_fixedData->m_infoList[index].m_gpsPvtVeln = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 16;
		m_fixedData->m_infoList[index].m_gpsPvtVele = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 20;
		m_fixedData->m_infoList[index].m_gpsPvtVeld = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 24;
		m_fixedData->m_infoList[index].m_gpsPvtHacc = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 28;
		m_fixedData->m_infoList[index].m_gpsPvtVacc = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 32;
		m_fixedData->m_infoList[index].m_gpsPvtSacc = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 36;
		m_fixedData->m_infoList[index].m_gpsPvtGpsAge = m_fixedData->m_infoList[index].m_gpsPvtData + 3 + 40;

		m_fixedData->m_infoList[index].m_size += (2+1) + (40 + 1);
	}
	// update
	m_msg.setDataShort(data.m_pressure, m_fixedData->m_infoList[index].m_gpsPvtPressure);
	m_msg.setDataByte(data.m_pressureAge, m_fixedData->m_infoList[index].m_gpsPvtPressureAge);

	// lon,lat,height,hacc,vacc,veln,vele,veld
	int32_t* bareln = (int32_t*)&data.m_itow;
	for (uint16_t i=0; i<10;++i)
		m_msg.setDataLong(bareln[i],m_fixedData->m_infoList[index].m_gpsPvtGpsData + (4*i));	//lint !e661 !e662

	// gpsAge
	m_msg.setDataByte(data.m_gpsAge,  m_fixedData->m_infoList[index].m_gpsPvtGpsAge);
	return true;
}

/*! \brief Return the pressure data component of a data item.
	\param index The index of the item of which the data should be returned.
	\returns The pressure data component of a data item.
*/
XsPressure LegacyDataPacket::pressure(int32_t index) const
{
	XsPressure buffer;
	if (containsPressure(index))
	{
		// pressure data
		// MH; \todo need a conversion factor here to go from short to double?
		buffer.m_pressure = (double)m_msg.getDataShort(m_fixedData->m_infoList[index].m_gpsPvtPressure);
		// pressAge
		buffer.m_pressureAge = m_msg.getDataByte(m_fixedData->m_infoList[index].m_gpsPvtPressureAge);
	}
	else if (containsMtwSdiData(index))
	{
		// pressure data
		buffer.m_pressure = (double)m_msg.getDataShort(m_fixedData->m_infoList[index].m_wBaroMeter);
		// pressAge
		buffer.m_pressureAge = (m_msg.getDataByte(m_fixedData->m_infoList[index].m_wAidingData)?0:255);
	}
	else
		buffer = XsPressure();
	return buffer;
}

/*!	\brief Return true if the packet contains pressure data
*/
bool LegacyDataPacket::containsPressure(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_gpsPvtPressure == XS_DATA_ITEM_NOT_AVAILABLE &&
		m_fixedData->m_infoList[index].m_wBaroMeter == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}

/*! \brief Add/update pressure data for the item
*/
bool LegacyDataPacket::setPressure(const XsPressure& data, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_gpsPvtPressure == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_gpsPvtData = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + (2+1));
		m_fixedData->m_infoList[index].m_gpsPvtPressure = m_fixedData->m_infoList[index].m_gpsPvtData;
		m_fixedData->m_infoList[index].m_gpsPvtPressureAge = m_fixedData->m_infoList[index].m_gpsPvtData + 2;

		m_fixedData->m_infoList[index].m_size += (2+1);
	}
	// update
	m_msg.setDataShort((uint16_t) data.m_pressure, m_fixedData->m_infoList[index].m_gpsPvtPressure);
	m_msg.setDataByte(data.m_pressureAge, m_fixedData->m_infoList[index].m_gpsPvtPressureAge);

	return true;
}

/*! \brief Return the strapdown integration (SDI) data component of a data item.
	\param index The index of the item of which the data should be returned.
	\returns The SDI data component of a data item.
*/
MtwSdiData LegacyDataPacket::mtwSdiData(int32_t index) const
{
	MtwSdiData buffer;
	if (containsMtwSdiData(index))
	{
		buffer.m_deviceId = m_msg.getDataLong(m_fixedData->m_infoList[index].m_wClientId);
		buffer.m_timeSync = m_msg.getDataByte(m_fixedData->m_infoList[index].m_wTimeSync);
		buffer.m_firstFrameNumber = m_msg.getDataShort(m_fixedData->m_infoList[index].m_wFirstFrameNumber);
		buffer.m_lastFrameNumber = m_msg.getDataShort(m_fixedData->m_infoList[index].m_wLastFrameNumber);
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_wCurrentBias), &buffer.m_currentBias[0], m_fixedData->m_infoList[index].m_wCurrentBias, 3);
		buffer.m_aidingData = (m_msg.getDataByte(m_fixedData->m_infoList[index].m_wAidingData)==1)?true:false;
		buffer.m_barometer = m_msg.getDataShort(m_fixedData->m_infoList[index].m_wBaroMeter) / 50.0;
		buffer.m_rssi = (int8_t) m_msg.getDataByte(m_fixedData->m_infoList[index].m_wRssi);

		m_msg.getDataFPValue(CHECKIFDOUBLE(m_wOrientationIncrement), &buffer.m_orientationIncrement[0], m_fixedData->m_infoList[index].m_wOrientationIncrement, 4);
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_wVelocityIncrement), &buffer.m_velocityIncrement[0], m_fixedData->m_infoList[index].m_wVelocityIncrement, 3);
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_wMagnetoMeter), &buffer.m_magnetoMeter[0], m_fixedData->m_infoList[index].m_wMagnetoMeter, 3);
	}
	else
	{
		buffer.m_deviceId = 0;
		buffer.m_timeSync = 0;
		buffer.m_firstFrameNumber = 0;
		buffer.m_lastFrameNumber = 0;
		buffer.m_currentBias.zero();
		buffer.m_aidingData = 0;
		buffer.m_barometer = 0;
		buffer.m_rssi = 0;

		buffer.m_orientationIncrement = XsQuaternion::identity();
		buffer.m_velocityIncrement.zero();
		buffer.m_magnetoMeter.zero();
	}
	return buffer;
}

/*! \brief Check if data item contains strapdown integration data
	\param index The index of the item of which the data should be returned.
	\returns true if the packet contains MTw SDI data
*/
bool LegacyDataPacket::containsMtwSdiData(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_mtwSdiData == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}

/*! \brief Add/update strapdown integration data for the item
	\param data The updated data
	\param index The index of the item of which the data should be returned.
	\returns true if the data was successfully updated
*/
bool LegacyDataPacket::setMtwSdiData(const MtwSdiData& data, int32_t index)
{
	if (m_fixedData->m_infoList[index].m_mtwSdiData == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_mtwSdiData = (uint16_t) m_msg.getDataSize();
		XsSize ds = 8;	// add as doubles
		m_msg.resizeData(m_msg.getDataSize() + 13 + 13*ds);

		m_fixedData->m_infoList[index].m_wClientId = m_fixedData->m_infoList[index].m_mtwSdiData;
		m_fixedData->m_infoList[index].m_wTimeSync = m_fixedData->m_infoList[index].m_mtwSdiData + 4;
		m_fixedData->m_infoList[index].m_wFirstFrameNumber = m_fixedData->m_infoList[index].m_mtwSdiData + 5;
		m_fixedData->m_infoList[index].m_wLastFrameNumber = m_fixedData->m_infoList[index].m_mtwSdiData + 7;
		m_fixedData->m_infoList[index].m_wCurrentBias = m_fixedData->m_infoList[index].m_mtwSdiData + 9;
		m_fixedData->m_infoList[index].m_wOrientationIncrement = (uint16_t) (m_fixedData->m_infoList[index].m_mtwSdiData + 9+3*ds);
		m_fixedData->m_infoList[index].m_wVelocityIncrement = (uint16_t) (m_fixedData->m_infoList[index].m_mtwSdiData + 9+7*ds);
		m_fixedData->m_infoList[index].m_wAidingData = (uint16_t) (m_fixedData->m_infoList[index].m_mtwSdiData + 9+10*ds);
		m_fixedData->m_infoList[index].m_wBaroMeter = (uint16_t) (m_fixedData->m_infoList[index].m_mtwSdiData + 10+10*ds);
		m_fixedData->m_infoList[index].m_wMagnetoMeter = (uint16_t) (m_fixedData->m_infoList[index].m_mtwSdiData + 12+10*ds);
		m_fixedData->m_infoList[index].m_wRssi = (uint16_t) (m_fixedData->m_infoList[index].m_mtwSdiData + 12+13*ds);
		m_fixedData->m_infoList[index].m_size += (uint16_t) (13 + 13*ds);
	}
	// update
	m_msg.setDataLong(data.m_deviceId.toInt(), m_fixedData->m_infoList[index].m_wClientId);
	m_msg.setDataByte(data.m_timeSync, m_fixedData->m_infoList[index].m_wTimeSync);
	m_msg.setDataShort(data.m_firstFrameNumber, m_fixedData->m_infoList[index].m_wFirstFrameNumber);
	m_msg.setDataShort(data.m_lastFrameNumber, m_fixedData->m_infoList[index].m_wLastFrameNumber);
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_wCurrentBias), data.m_currentBias.data(), m_fixedData->m_infoList[index].m_wCurrentBias, 3);
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_wOrientationIncrement), data.m_orientationIncrement.data(), m_fixedData->m_infoList[index].m_wOrientationIncrement, 4);
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_wVelocityIncrement), data.m_velocityIncrement.data(), m_fixedData->m_infoList[index].m_wVelocityIncrement, 3);
	m_msg.setDataByte((data.m_aidingData==true)?1:0, m_fixedData->m_infoList[index].m_wAidingData);
	m_msg.setDataShort((uint16_t)(XsMath_doubleToLong(data.m_barometer * 50)), m_fixedData->m_infoList[index].m_wBaroMeter);
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_wMagnetoMeter), data.m_magnetoMeter.data(), m_fixedData->m_infoList[index].m_wMagnetoMeter, 3);
	m_msg.setDataByte(data.m_rssi, m_fixedData->m_infoList[index].m_wRssi);

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Temperature component of a data item.
double LegacyDataPacket::temperature(int32_t index, int channel) const
{
	if (containsTemperature(index, channel))
		return m_msg.getDataFPValue(CHECKIFDOUBLE(m_temp[channel]), m_fixedData->m_infoList[index].m_temp[channel]);

	return 0.0;
}
bool LegacyDataPacket::containsTemperature(int32_t index, int channel) const
{
	if (dataSize(index) == 0)
		return false;
	if (channel >= XS_MAX_TEMPERATURE_CHANNELS)
		return false;
	if (m_fixedData->m_infoList[index].m_temp[channel] == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setTemperature(const double& temp, int32_t index, int channel)
{
	if (dataSize(index) == 0)
		return false;
	if (channel >= XS_MAX_TEMPERATURE_CHANNELS)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_temp[channel] == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_temp[channel]))
	{
		// add
		ds = 8;
		//m_msg.m_autoUpdateChecksum = false;
		int c = temperatureChannelCount(index);
		for(int i = c; i <= channel; i++) //Add (also add missing intermediate channels)
		{
			m_fixedData->m_infoList[index].m_temp[i] = (uint16_t) m_msg.getDataSize();
			m_msg.resizeData(m_msg.getDataSize() + ds);
			m_fixedData->m_infoList[index].m_size += ds;
		}
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_temp[channel]), temp, m_fixedData->m_infoList[index].m_temp[channel]);
	return true;
}
int LegacyDataPacket::temperatureChannelCount(int32_t index) const
{
	for(int i = 0; i < XS_MAX_TEMPERATURE_CHANNELS; i++)
	{
		if(m_fixedData->m_infoList[index].m_temp[i] == XS_DATA_ITEM_NOT_AVAILABLE)
			return i;
	}
	return XS_MAX_TEMPERATURE_CHANNELS;
}
//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Accelerometer component of a data item.
XsVector LegacyDataPacket::calibratedAcceleration(int32_t index) const
{
	XsVector3 buffer;
	if (containsCalibratedAcceleration(index))
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_calAcc), &buffer[0], m_fixedData->m_infoList[index].m_calAcc, 3);
	else
		buffer.zero();
	return buffer;
}
bool LegacyDataPacket::containsCalibratedAcceleration(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_calAcc == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setCalibratedAcceleration(const XsVector& vec, int32_t index)
{
	const uint16_t numValues = 3;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_calAcc == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_calAcc))
	{
		// add
		ds = 8;
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_calAcc = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_calAcc), vec.data(), m_fixedData->m_infoList[index].m_calAcc, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Gyroscope component of a data item.
XsVector LegacyDataPacket::calibratedGyroscopeData(int32_t index) const
{
	XsVector3 buffer;
	if (containsCalibratedGyroscopeData(index))
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_calGyr), &buffer[0], m_fixedData->m_infoList[index].m_calGyr, 3);
	else
		buffer.zero();
	return buffer;
}
bool LegacyDataPacket::containsCalibratedGyroscopeData(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_calGyr == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setCalibratedGyroscopeData(const XsVector& vec, int32_t index)
{
	const uint16_t numValues = 3;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_calGyr == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_calGyr))
	{
		// add
		ds = 8;
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_calGyr = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_calGyr), vec.data(), m_fixedData->m_infoList[index].m_calGyr, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Magnetometer component of a data item.
XsVector LegacyDataPacket::calibratedMagneticField(int32_t index) const
{
	XsVector3 buffer;
	if (containsCalibratedMagneticField(index))
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_calMag), &buffer[0], m_fixedData->m_infoList[index].m_calMag, 3);
	else
		buffer.zero();
	return buffer;
}
bool LegacyDataPacket::containsCalibratedMagneticField(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_calMag == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setCalibratedMagneticField(const XsVector& vec, int32_t index)
{
	const uint16_t numValues = 3;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_calMag == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_calMag))
	{
		// add
		ds = 8;
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_calMag = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_calMag), vec.data(), m_fixedData->m_infoList[index].m_calMag, numValues);
	return true;
}

/*! \brief Return the Calibrated Data component of a data item.
	\param index The index of the item of which the data should be returned.
	\returns The Calibrated Data component of a data item.
*/
XsCalibratedData LegacyDataPacket::calibratedData(int32_t index) const
{
	XsCalibratedData buffer;
	if (containsCalibratedData(index))
	{
		//lint --e{419}
		double* bare = &buffer.m_acc[0];
		if (m_fixedData->m_infoList[index].m_calAcc == XS_DATA_ITEM_NOT_AVAILABLE)
			memset(bare, 0, 3*sizeof(double));
		else
			m_msg.getDataFPValue(CHECKIFDOUBLE(m_calAcc), bare, m_fixedData->m_infoList[index].m_calAcc, 3);

		bare = &buffer.m_gyr[0];
		if (m_fixedData->m_infoList[index].m_calGyr == XS_DATA_ITEM_NOT_AVAILABLE)
			memset(bare, 0, 3*sizeof(double));
		else
			m_msg.getDataFPValue(CHECKIFDOUBLE(m_calGyr), bare, m_fixedData->m_infoList[index].m_calGyr, 3);

		bare = &buffer.m_mag[0];
		if (m_fixedData->m_infoList[index].m_calMag == XS_DATA_ITEM_NOT_AVAILABLE)
			memset(bare, 0, 3*sizeof(double));
		else
			m_msg.getDataFPValue(CHECKIFDOUBLE(m_calMag), bare, m_fixedData->m_infoList[index].m_calMag, 3);
	}
	else
	{
		buffer.m_acc.zero();
		buffer.m_gyr.zero();
		buffer.m_mag.zero();
	}
	return buffer;
}

/*! \brief Check if data item contains Calibrated Data
	\param index The index of the item of which the data should be returned.
	\returns true if the packet contains Calibrated Data
*/
bool LegacyDataPacket::containsCalibratedData(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	return (m_fixedData->m_infoList[index].m_calData != XS_DATA_ITEM_NOT_AVAILABLE);
}

/*! \brief Add/update Calibrated Data for the item */
bool LegacyDataPacket::setCalibratedData(const XsCalibratedData& data, int32_t index)
{
	const uint16_t numValues = 9;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_calData == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_calData))
	{
		// add
		ds = 8;	// added values are always in double precision
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_calData = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_calAcc = m_fixedData->m_infoList[index].m_calData;
		m_fixedData->m_infoList[index].m_calGyr = m_fixedData->m_infoList[index].m_calData + 3*ds;
		m_fixedData->m_infoList[index].m_calMag = m_fixedData->m_infoList[index].m_calData + 6*ds;
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	double* bare = (double*) &data;
	if (m_fixedData->m_infoList[index].m_calAcc != XS_DATA_ITEM_NOT_AVAILABLE)
		m_msg.setDataFPValue(CHECKIFDOUBLE(m_calAcc), bare, m_fixedData->m_infoList[index].m_calAcc, 3);
	bare += 3;
	if (m_fixedData->m_infoList[index].m_calGyr != XS_DATA_ITEM_NOT_AVAILABLE)
		m_msg.setDataFPValue(CHECKIFDOUBLE(m_calGyr), bare, m_fixedData->m_infoList[index].m_calGyr, 3);
	bare += 3;
	if (m_fixedData->m_infoList[index].m_calMag != XS_DATA_ITEM_NOT_AVAILABLE)
		m_msg.setDataFPValue(CHECKIFDOUBLE(m_calMag), bare, m_fixedData->m_infoList[index].m_calMag, 3);

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as a Quaternion.
XsQuaternion LegacyDataPacket::orientationQuaternion(int32_t index) const
{
	XsQuaternion buffer;
	if (containsOrientationQuaternion(index))
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_oriQuat), &buffer[0], m_fixedData->m_infoList[index].m_oriQuat, 4);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}

bool LegacyDataPacket::containsOrientationQuaternion(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_oriQuat == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setOrientationQuaternion(const XsQuaternion& data, int32_t index)
{
	const uint16_t numValues = 4;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_oriQuat == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_oriQuat))
	{
		// add
		ds = 8;	// added values are always in double precision
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_oriQuat = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;

		double* bare = (double*) &data;
		m_msg.setDataFPValue((m_fixedData->m_formatList[index].m_outputSettings & XOS_Dataformat_Mask)|XOS_Dataformat_Double,
							bare, m_fixedData->m_infoList[index].m_oriQuat, numValues);
		return true;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_oriQuat), data.data(), m_fixedData->m_infoList[index].m_oriQuat, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as XsEuler angles.
XsEuler LegacyDataPacket::orientationEuler(int32_t index) const
{
	XsEuler buffer;
	if (containsOrientationEuler(index))
	{

		m_msg.getDataFPValue(CHECKIFDOUBLE(m_oriEul), &buffer[0], m_fixedData->m_infoList[index].m_oriEul, 3);
	}
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool LegacyDataPacket::containsOrientationEuler(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_oriEul == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setOrientationEuler(const XsEuler& data, int32_t index)
{
	const uint16_t numValues = 3;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_oriEul == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_oriEul))
	{
		// add
		ds = 8;	// added values are always in double precision
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_oriEul = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_oriEul), data.data(), m_fixedData->m_infoList[index].m_oriEul, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as an Orientation Matrix.
XsMatrix LegacyDataPacket::orientationMatrix(int32_t index) const
{
	XsMatrix3x3 buffer;
	uint16_t k = 0;
	if (containsOrientationMatrix(index))
	{
		// remember to use column major order in the xbus message!
		uint16_t ds = getFPValueSize(index);
		for (int32_t i=0;i<3;++i)
			for (int32_t j=0;j<3;++j, k+=ds)
				buffer.setValue(j, i, m_msg.getDataFPValue(CHECKIFDOUBLE(m_oriMat), m_fixedData->m_infoList[index].m_oriMat+k));
	}
	else
		buffer.zero();

	return buffer;
}
bool LegacyDataPacket::containsOrientationMatrix(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_oriMat == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setOrientationMatrix(const XsMatrix& data, int32_t index)
{
	const uint16_t numValues = 9;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	// remember to use column major order in the xbus message!
	if (m_fixedData->m_infoList[index].m_oriMat == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_oriMat))
	{
		// add
		ds = 8;	// added values are always in double precision
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_oriMat = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	uint16_t k = 0;
	for (int32_t i=0;i<3;++i)
		for (int32_t j=0;j<3;++j, k+=ds)
			m_msg.setDataFPValue(CHECKIFDOUBLE(m_oriMat), data.value(j, i), m_fixedData->m_infoList[index].m_oriMat+k);
	return true;
}

bool LegacyDataPacket::containsOrientation(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_oriEul == XS_DATA_ITEM_NOT_AVAILABLE &&
		m_fixedData->m_infoList[index].m_oriMat == XS_DATA_ITEM_NOT_AVAILABLE &&
		m_fixedData->m_infoList[index].m_oriQuat == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the AnalogIn 1 component of a data item.
XsAnalogInData LegacyDataPacket::analogIn1Data(int32_t index) const
{
	XsAnalogInData buffer;
	if (containsAnalogIn1Data(index))
		buffer.m_data = m_msg.getDataShort(m_fixedData->m_infoList[index].m_analogIn1);

	return buffer;
}
bool LegacyDataPacket::containsAnalogIn1Data(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_analogIn1 == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setAnalogIn1Data(const XsAnalogInData& data, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_analogIn1 == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_analogIn1 = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_fixedData->m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(data.m_data, m_fixedData->m_infoList[index].m_analogIn1);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the AnalogIn 2 component of a data item.
XsAnalogInData LegacyDataPacket::analogIn2Data(int32_t index) const
{
	XsAnalogInData buffer;
	if (containsAnalogIn2Data(index))
		buffer.m_data = m_msg.getDataShort(m_fixedData->m_infoList[index].m_analogIn2);

	return buffer;
}
bool LegacyDataPacket::containsAnalogIn2Data(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_analogIn2 == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setAnalogIn2Data(const XsAnalogInData& data, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_analogIn2 == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_analogIn2 = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_fixedData->m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(data.m_data, m_fixedData->m_infoList[index].m_analogIn2);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Position LLA component of a data item.
XsVector LegacyDataPacket::positionLLA(int32_t index) const
{
	XsVector3 buffer;
	if (containsPositionLLA(index))
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_posLLA), &buffer[0], m_fixedData->m_infoList[index].m_posLLA, 3);
	else
		buffer.zero();
	return buffer;
}
bool LegacyDataPacket::containsPositionLLA(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_posLLA == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setPositionLLA(const XsVector& data, int32_t index)
{
	const uint16_t numValues = 3;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_posLLA == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_posLLA))
	{
		// add
		ds = 8;	// added values are always in double precision
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_posLLA = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_posLLA), data.data(), m_fixedData->m_infoList[index].m_posLLA, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Velocity NED component of a data item.
XsVector LegacyDataPacket::velocity(int32_t index) const
{
	XsVector3 buffer;
	if (containsVelocity(index))
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_velNEDorNWU), &buffer[0], m_fixedData->m_infoList[index].m_velNEDorNWU, 3);
	else
		buffer.zero();
	return buffer;
}
bool LegacyDataPacket::containsVelocity(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_velNEDorNWU == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setVelocity(const XsVector& data, int32_t index)
{
	const uint16_t numValues = 3;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_velNEDorNWU == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_posLLA))
	{
		// add
		ds = 8;	// added values are always in double precision
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_velNEDorNWU = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_velNEDorNWU), data.data(), m_fixedData->m_infoList[index].m_velNEDorNWU, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Status component of a data item.
uint32_t LegacyDataPacket::status(int32_t index, bool* outIsDetailed) const
{
	assert(outIsDetailed != 0);
	if (containsStatus(index))
	{
		if (m_fixedData->m_infoList[index].m_detailedStatus == XS_DATA_ITEM_NOT_AVAILABLE)
		{
			*outIsDetailed = false;
			return m_msg.getDataByte(m_fixedData->m_infoList[index].m_status);
		}
		else
		{
			*outIsDetailed = true;
			uint32_t tmp = m_msg.getDataLong(m_fixedData->m_infoList[index].m_detailedStatus);
			return tmp;
		}
	}
	else
	{
		return 0;
	}
}
bool LegacyDataPacket::containsStatus(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_status == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::containsDetailedStatus(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_detailedStatus == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setStatus(const uint32_t data, int32_t index)
{
	if (dataSize(index) == 0)
		return false;

	if (m_fixedData->m_infoList[index].m_status == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_status = (uint16_t) m_msg.getDataSize();
		m_fixedData->m_infoList[index].m_detailedStatus = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 4);
		m_fixedData->m_infoList[index].m_size += 4;
	}
	// update
	if (m_fixedData->m_infoList[index].m_detailedStatus == XS_DATA_ITEM_NOT_AVAILABLE)
		m_msg.setDataByte(data&0xFF, m_fixedData->m_infoList[index].m_status);
	else
		m_msg.setDataLong(data, m_fixedData->m_infoList[index].m_detailedStatus);

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Sample Counter component of the packet.
uint16_t LegacyDataPacket::packetCounter(int32_t index) const
{
	if (!containsPacketCounter(index))
		return 0;
	return m_msg.getDataShort(m_fixedData->m_infoList[index].m_sc);
}
bool LegacyDataPacket::containsPacketCounter(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_sc == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setPacketCounter(const uint16_t counter, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_sc == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_sc = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_fixedData->m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(counter, m_fixedData->m_infoList[index].m_sc);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the UTC Time component of the packet.
XsUtcTime LegacyDataPacket::utcTime(int32_t index) const
{
	XsUtcTime buffer;
	if (containsUtcTime(index))
	{
		buffer.m_nano = m_msg.getDataLong(m_fixedData->m_infoList[index].m_utcNano);
		buffer.m_year = m_msg.getDataShort(m_fixedData->m_infoList[index].m_utcYear);

		// month, day, hour, minute, second and valid
		uint8_t *bareByte = (uint8_t*) &buffer.m_month;
		for (uint16_t i=0; i < 6; ++i)
			bareByte[i] = m_msg.getDataByte(m_fixedData->m_infoList[index].m_utcMonth + i);	//lint !e661 !e662
	}
	else
		buffer = XsUtcTime();
	return buffer;
}
bool LegacyDataPacket::containsUtcTime(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_utcTime == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setUtcTime(const XsUtcTime& data, int32_t index)
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_utcTime == XS_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_fixedData->m_infoList[index].m_utcTime = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 12);
		m_fixedData->m_infoList[index].m_utcNano = m_fixedData->m_infoList[index].m_utcTime;
		m_fixedData->m_infoList[index].m_utcYear = m_fixedData->m_infoList[index].m_utcTime + 4;
		m_fixedData->m_infoList[index].m_utcMonth = m_fixedData->m_infoList[index].m_utcTime + 6;
		m_fixedData->m_infoList[index].m_utcDay = m_fixedData->m_infoList[index].m_utcTime + 7;
		m_fixedData->m_infoList[index].m_utcHour = m_fixedData->m_infoList[index].m_utcTime + 8;
		m_fixedData->m_infoList[index].m_utcMinute = m_fixedData->m_infoList[index].m_utcTime + 9;
		m_fixedData->m_infoList[index].m_utcSecond = m_fixedData->m_infoList[index].m_utcTime + 10;
		m_fixedData->m_infoList[index].m_utcValid = m_fixedData->m_infoList[index].m_utcTime + 11;

		m_fixedData->m_infoList[index].m_size += 12;
	}
	// update
	m_msg.setDataLong(data.m_nano, m_fixedData->m_infoList[index].m_utcNano);
	m_msg.setDataShort(data.m_year, m_fixedData->m_infoList[index].m_utcYear);

	// month, day, hour, minute, second and valid
	int8_t* bareByte = (int8_t*)&data.m_month;
	for (uint16_t i=0; i<6;++i)
		m_msg.setDataByte(bareByte[i],m_fixedData->m_infoList[index].m_utcMonth + i);	//lint !e661 !e662

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the XKF-3 Acc G component of the packet.
XsVector LegacyDataPacket::freeAcceleration(int32_t index) const
{
	XsVector3 buffer;
	if (containsFreeAcceleration(index))
		m_msg.getDataFPValue(CHECKIFDOUBLE(m_acc_g), &buffer[0], m_fixedData->m_infoList[index].m_acc_g, 3);
	else
		buffer.zero();
	return buffer;
}
bool LegacyDataPacket::containsFreeAcceleration(int32_t index) const
{
	if (dataSize(index) == 0)
		return false;
	if (m_fixedData->m_infoList[index].m_acc_g == XS_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool LegacyDataPacket::setFreeAcceleration(const XsVector& g, int32_t index)
{
	const uint16_t numValues = 3;
	if (dataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_fixedData->m_infoList[index].m_acc_g == XS_DATA_ITEM_NOT_AVAILABLE || !ISDOUBLE(m_acc_g))
	{
		// add
		ds = 8;	// added values are always in double precision
		//m_msg.m_autoUpdateChecksum = false;

		m_fixedData->m_infoList[index].m_acc_g = (uint16_t) m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_fixedData->m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(CHECKIFDOUBLE(m_acc_g), g.data(), m_fixedData->m_infoList[index].m_acc_g, numValues);
	return true;
}

XsTimeStamp LegacyDataPacket::triggerIndication( int channelID ) const
{
	switch(channelID)
	{
	case 1:
		return m_triggerIn1;
	case 2:
		return m_triggerIn2;

	default:
		return XsTimeStamp();
	}
}

bool LegacyDataPacket::containsTriggerIndication( int channelID /*= 0*/ ) const
{
	switch(channelID)
	{
	case 0:
		return (m_triggerIn1.msTime() == 0 && m_triggerIn2.msTime() == 0) ? false : true;
	case 1:
		return (m_triggerIn1.msTime() == 0) ? false : true;
	case 2:
		return (m_triggerIn2.msTime() == 0) ? false : true;

	default:
		return false;
	}
}

bool LegacyDataPacket::setTriggerIndication( int channelID, const XsTimeStamp &t )
{
	switch(channelID)
	{
	case 1:
		m_triggerIn1 = t;
		return true;
	case 2:
		m_triggerIn2 = t;
		return true;
	}

	return false;
}
//lint +esym(613, LegacyDataPacket::m_fixedData)
