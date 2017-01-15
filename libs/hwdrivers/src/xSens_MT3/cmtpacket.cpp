/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*! \file Cmtpacket.cpp

	For information about objects in this file, see the appropriate header:
	\ref Cmtpacket.h

	\section FileCopyright Copyright Notice
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.

	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
*/

#include "cmtpacket.h"

#ifdef _CMT_DLL_EXPORT
#include "xsens_math.h"
#endif

#ifdef _LOG_PACKET
#	define PACKETLOG	CMTLOG
#else
#	define PACKETLOG(...)
#endif


namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Packet class //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
Packet::Packet(uint16_t items, bool xbus)
{
	PACKETLOG("Create Packet %p\n",this);

	m_itemCount = items;
	m_infoList = NULL;
	m_formatList = new CmtDataFormat[m_itemCount];
	m_toa = 0;
	m_rtc = 0;
	m_xm = xbus;
}

//////////////////////////////////////////////////////////////////////////////////////////
Packet::~Packet()
{
	PACKETLOG("Destroy Packet %p\n",this);
	LSTCHKDELNUL(m_formatList);
	LSTCHKDELNUL(m_infoList);
}

//////////////////////////////////////////////////////////////////////////////////////////
CmtDataFormat Packet::getDataFormat(const uint16_t index) const
{
	if (index >= m_itemCount || m_formatList == NULL)
	{
		CmtDataFormat temp;
		return temp;
	}
	return m_formatList[index];
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Packet::setDataFormat(const CmtDataFormat& format, const uint16_t index)
{
	if (index < m_itemCount)
	{
		m_formatList[index] = format;
		LSTCHKDELNUL(m_infoList);
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Packet::setDataFormat(const CmtOutputMode outputMode, const CmtOutputSettings outputSettings, const uint16_t index)
{
	if (index < m_itemCount)
	{
		m_formatList[index].m_outputMode = outputMode;
		m_formatList[index].m_outputSettings = outputSettings;
		LSTCHKDELNUL(m_infoList);
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Packet::getXbus(void) const
{
	return m_xm;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Packet::setXbus(bool xbus, bool convert)
{
	if (xbus != m_xm)
	{
		if (convert)
		{
			CmtMtTimeStamp stamp = getSampleCounter(0);

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
		m_xm = xbus;
		LSTCHKDELNUL(m_infoList);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the floating/fixed point value size
uint16_t Packet::getFPValueSize(const uint16_t index) const
{
	uint16_t ds;
	switch (m_formatList[index].m_outputSettings & CMT_OUTPUTSETTINGS_DATAFORMAT_MASK)
	{
		case CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT:
			ds = 4;
			break;

		case CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE:
			ds = 8;
			break;
		case CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632:
			ds = 6;
			break;

		case CMT_OUTPUTSETTINGS_DATAFORMAT_F1220:
			ds = 4;
			break;

        default:
            ds = 0;
            break;
	}
	return ds;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the data size.
uint16_t Packet::getDataSize(const uint16_t index) const
{
	if (m_infoList == NULL)
	{
		// allocate list
		m_infoList = new PacketInfo[m_itemCount];
		uint16_t totalOffset;
		if (m_xm)
			totalOffset = 2;
		else
			totalOffset = 0;

		// fill lists
		for (uint16_t i = 0;i < m_itemCount;++i)
		{
			m_infoList[i].m_offset = totalOffset;
			m_infoList[i].m_size = 0;

			uint16_t ds = getFPValueSize(i);

			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_RAW)
			{
				m_infoList[i].m_rawData = totalOffset;
				m_infoList[i].m_rawAcc = totalOffset;
				m_infoList[i].m_rawGyr = totalOffset+6;
				m_infoList[i].m_rawMag = totalOffset+12;
				m_infoList[i].m_rawTemp = totalOffset+18;

				m_infoList[i].m_size += 20;
				totalOffset += 20;
			}
			else
			{
				m_infoList[i].m_rawData = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawAcc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGyr = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawMag = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawTemp = CMT_DATA_ITEM_NOT_AVAILABLE;
			}


			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_RAWGPSPRINT)
			{
				m_infoList[i].m_rawGpsData = totalOffset;
				m_infoList[i].m_rawGpsPressure = totalOffset;
				m_infoList[i].m_rawGpsPressureAge = totalOffset + 2;
				m_infoList[i].m_size += 3;
				totalOffset += 3;

				m_infoList[i].m_rawGpsGpsData = totalOffset;
				m_infoList[i].m_rawGpsItow = totalOffset;
				m_infoList[i].m_rawGpsLatitude = totalOffset + 4;
				m_infoList[i].m_rawGpsLongitude = totalOffset + 8;
				m_infoList[i].m_rawGpsHeight = totalOffset + 12;
				m_infoList[i].m_rawGpsVeln = totalOffset + 16;
				m_infoList[i].m_rawGpsVele = totalOffset + 20;
				m_infoList[i].m_rawGpsVeld = totalOffset + 24;
				m_infoList[i].m_rawGpsHacc = totalOffset + 28;
				m_infoList[i].m_rawGpsVacc = totalOffset + 32;
				m_infoList[i].m_rawGpsSacc = totalOffset + 36;
				m_infoList[i].m_rawGpsGpsAge = totalOffset + 40;
				m_infoList[i].m_size += 41;
				totalOffset += 41;
			}
			else
			{
				m_infoList[i].m_rawGpsData = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsPressure = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsPressureAge = CMT_DATA_ITEM_NOT_AVAILABLE;

				m_infoList[i].m_rawGpsGpsData = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsItow = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsLatitude = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsLongitude = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsHeight = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsVeln = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsVele = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsVeld = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsHacc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsVacc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsSacc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_rawGpsGpsAge = CMT_DATA_ITEM_NOT_AVAILABLE;
			}

			m_infoList[i].m_temp = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_TEMP)
			{
				m_infoList[i].m_temp = totalOffset;
				m_infoList[i].m_size += ds;
				totalOffset += ds;
			}

			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_CALIB)
			{
				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_CALIBMODE_ACC_MASK) == 0)
				{
					m_infoList[i].m_calAcc = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
				else
					m_infoList[i].m_calAcc = CMT_DATA_ITEM_NOT_AVAILABLE;

				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_CALIBMODE_GYR_MASK) == 0)
				{
					m_infoList[i].m_calGyr = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
				else
					m_infoList[i].m_calGyr = CMT_DATA_ITEM_NOT_AVAILABLE;

				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_CALIBMODE_MAG_MASK) == 0)
				{
					m_infoList[i].m_calMag = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
				else
					m_infoList[i].m_calMag = CMT_DATA_ITEM_NOT_AVAILABLE;

				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYRMAG) == 0)
					m_infoList[i].m_calData = m_infoList[i].m_calAcc;
				else
					m_infoList[i].m_calData = CMT_DATA_ITEM_NOT_AVAILABLE;
			}
			else
			{
				m_infoList[i].m_calData = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_calAcc = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_calGyr = CMT_DATA_ITEM_NOT_AVAILABLE;
				m_infoList[i].m_calMag = CMT_DATA_ITEM_NOT_AVAILABLE;
			}

			m_infoList[i].m_oriEul = CMT_DATA_ITEM_NOT_AVAILABLE;
			m_infoList[i].m_oriQuat = CMT_DATA_ITEM_NOT_AVAILABLE;
			m_infoList[i].m_oriMat = CMT_DATA_ITEM_NOT_AVAILABLE;

			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_ORIENT)
			{
				switch (m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK)
				{
				case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
					m_infoList[i].m_oriEul = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
					break;
				case CMT_OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
					m_infoList[i].m_oriQuat = totalOffset;
					m_infoList[i].m_size += 4*ds;
					totalOffset += 4*ds;
					break;
				case CMT_OUTPUTSETTINGS_ORIENTMODE_MATRIX:
					m_infoList[i].m_oriMat = totalOffset;
					m_infoList[i].m_size += 9*ds;
					totalOffset += 9*ds;
					break;
				default:
					break;
				}
			}

			m_infoList[i].m_analogIn1 = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_AUXILIARY)
			{
				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN1_MASK) == 0)
				{
					m_infoList[i].m_analogIn1 = totalOffset;
					m_infoList[i].m_size += 2;
					totalOffset += 2;
				}
				else
					m_infoList[i].m_analogIn1 = CMT_DATA_ITEM_NOT_AVAILABLE;

				if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_AUXILIARYMODE_AIN2_MASK) == 0)
				{
					m_infoList[i].m_analogIn2 = totalOffset;
					m_infoList[i].m_size += 2;
					totalOffset += 2;
				}
				else
					m_infoList[i].m_analogIn2 = CMT_DATA_ITEM_NOT_AVAILABLE;
			}

			m_infoList[i].m_posLLA = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_POSITION)
			{
				switch (m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_POSITIONMODE_MASK)
				{
				case CMT_OUTPUTSETTINGS_POSITIONMODE_LLA_WGS84:
					m_infoList[i].m_posLLA = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
			}

			m_infoList[i].m_velNEDorNWU = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_VELOCITY)
			{
				switch (m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_VELOCITYMODE_MASK)
				{
				case CMT_OUTPUTSETTINGS_VELOCITYMODE_NED:
					m_infoList[i].m_velNEDorNWU = totalOffset;
					m_infoList[i].m_size += 3*ds;
					totalOffset += 3*ds;
				}
			}

			m_infoList[i].m_status = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_formatList[i].m_outputMode & CMT_OUTPUTMODE_STATUS)
			{
				m_infoList[i].m_status = totalOffset;
				m_infoList[i].m_size += 1;
				totalOffset += 1;
			}

			m_infoList[i].m_sc = CMT_DATA_ITEM_NOT_AVAILABLE;
			if (m_xm)
				m_infoList[i].m_sc = 0;
			if ((m_formatList[i].m_outputSettings & CMT_OUTPUTSETTINGS_TIMESTAMP_MASK) ==
					CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT)
			{
				if (!m_xm)
					m_infoList[i].m_sc = totalOffset;
				m_infoList[i].m_size += 2;
				totalOffset += 2;
			}

			// post-processing data is never available at this point
			m_infoList[i].m_acc_g = CMT_DATA_ITEM_NOT_AVAILABLE;
		}
	}

	if (index < m_itemCount)
		return m_infoList[index].m_size;
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Accelerometer component of a data item.
CmtShortVector Packet::getRawAcc(const uint16_t index) const
{
	CmtShortVector buffer;
	if (containsRawAcc(index))
		for (uint16_t i=0;i<3;++i)
			buffer.m_data[i] = m_msg.getDataShort(m_infoList[index].m_rawAcc + (2*i));

	return buffer;
}
bool Packet::containsRawAcc(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawAcc == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawAcc(const CmtShortVector& vec, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawAcc == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawAcc = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec.m_data[i], m_infoList[index].m_rawAcc + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Gyroscope component of a data item.
CmtShortVector Packet::getRawGyr(const uint16_t index) const
{
	CmtShortVector buffer;
	if (containsRawGyr(index))
		for (uint16_t i=0;i<3;++i)
			buffer.m_data[i] = m_msg.getDataShort(m_infoList[index].m_rawGyr + (2*i));

	return buffer;
}
bool Packet::containsRawGyr(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawGyr == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawGyr(const CmtShortVector& vec, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawGyr == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawGyr = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec.m_data[i], m_infoList[index].m_rawGyr + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Magnetometer component of a data item.
CmtShortVector Packet::getRawMag(const uint16_t index) const
{
	CmtShortVector buffer;
	if (containsRawMag(index))
		for (uint16_t i=0;i<3;++i)
			buffer.m_data[i] = m_msg.getDataShort(m_infoList[index].m_rawMag + (2*i));

	return buffer;
}
bool Packet::containsRawMag(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawMag == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawMag(const CmtShortVector& vec, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawMag == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawMag = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3*2);
		m_infoList[index].m_size += 3*2;
	}
	// update
	for (uint16_t i=0;i<3;++i)
		m_msg.setDataShort(vec.m_data[i], m_infoList[index].m_rawMag + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Temperature component of a data item.
uint16_t Packet::getRawTemp(const uint16_t index) const
{
	if (!containsRawTemp(index))
		return 0;

	return m_msg.getDataShort(m_infoList[index].m_rawTemp);
}
bool Packet::containsRawTemp(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawTemp == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawTemp(const uint16_t temp, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawTemp == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawTemp = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(temp, m_infoList[index].m_rawTemp);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Data component of a data item.
CmtRawData Packet::getRawData(const uint16_t index) const
{
	CmtRawData buffer;
	if (containsRawData(index))
	{
		const uint8_t* tmp = m_msg.getDataBuffer(m_infoList[index].m_rawData);
		const uint16_t* sh = (const uint16_t*) tmp;
		uint16_t* bare = (uint16_t*) &buffer;

		for (uint16_t i=0;i<10;++i, ++sh, ++bare)
			*bare = swapEndian16(*sh);// m_msg.getDataShort(m_infoList[index].m_rawData + (2*i));
	}
	return buffer;
}
bool Packet::containsRawData(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawData == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawData(const CmtRawData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawData == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawData = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 3* 3*2 + 2);
		m_infoList[index].m_rawAcc = m_infoList[index].m_rawData;
		m_infoList[index].m_rawGyr = m_infoList[index].m_rawData + 3*2;
		m_infoList[index].m_rawMag = m_infoList[index].m_rawData + 6*2;
		m_infoList[index].m_rawTemp= m_infoList[index].m_rawData + 9*2;
		m_infoList[index].m_size += 3* 3*2 + 2;
	}
	// update
	int16_t* bare = (int16_t*) &data;
	for (uint16_t i=0;i<10;++i)
		m_msg.setDataShort(bare[i], m_infoList[index].m_rawData + (2*i));
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Raw Data component of a data item.
CmtRawGpsData Packet::getRawGpsData(const uint16_t index) const
{
	CmtRawGpsData buffer;
	if (containsRawGpsData(index))
	{
		//const uint8_t* tmp = m_msg.getDataBuffer(m_infoList[index].m_rawGpsData);
		//const uint16_t* sh = (const uint16_t*) tmp;
		//uint16_t* bare = (uint16_t*) &buffer;

		// pressure data
		buffer.m_pressure = m_msg.getDataShort(m_infoList[index].m_rawGpsPressure);
		// pressAge
		buffer.m_pressureAge = m_msg.getDataByte(m_infoList[index].m_rawGpsPressureAge);

		// lon,lat,height,hacc,vacc,veln,vele,veld
		//tmp = m_msg.getDataBuffer(m_infoList[index].m_rawGpsGpsData);
		//const uint32_t* ln = (const uint32_t*) tmp;
		uint32_t *bareln = (uint32_t*) &buffer.m_itow;
		for (uint16_t i=0; i < 10; ++i)
			bareln[i] = m_msg.getDataLong(m_infoList[index].m_rawGpsGpsData + (4*i));

		// gpsAge
		buffer.m_gpsAge = m_msg.getDataByte(m_infoList[index].m_rawGpsGpsAge);
	}
	return buffer;
}
bool Packet::containsRawGpsData(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawGpsData == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateRawGpsData(const CmtRawGpsData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_rawGpsData == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_rawGpsData = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + (2+1) + (40 + 1));
		m_infoList[index].m_rawGpsPressure = m_infoList[index].m_rawGpsData;
		m_infoList[index].m_rawGpsPressureAge = m_infoList[index].m_rawGpsData + 2;

		m_infoList[index].m_rawGpsGpsData = m_infoList[index].m_rawGpsData + 3;
		m_infoList[index].m_rawGpsItow = m_infoList[index].m_rawGpsData + 3;
		m_infoList[index].m_rawGpsLatitude = m_infoList[index].m_rawGpsData + 3 + 4;
		m_infoList[index].m_rawGpsLongitude = m_infoList[index].m_rawGpsData + 3 + 8;
		m_infoList[index].m_rawGpsHeight = m_infoList[index].m_rawGpsData + 3 + 12;
		m_infoList[index].m_rawGpsVeln = m_infoList[index].m_rawGpsData + 3 + 16;
		m_infoList[index].m_rawGpsVele = m_infoList[index].m_rawGpsData + 3 + 20;
		m_infoList[index].m_rawGpsVeld = m_infoList[index].m_rawGpsData + 3 + 24;
		m_infoList[index].m_rawGpsHacc = m_infoList[index].m_rawGpsData + 3 + 28;
		m_infoList[index].m_rawGpsVacc = m_infoList[index].m_rawGpsData + 3 + 32;
		m_infoList[index].m_rawGpsSacc = m_infoList[index].m_rawGpsData + 3 + 36;
		m_infoList[index].m_rawGpsGpsAge = m_infoList[index].m_rawGpsData + 3 + 40;

		m_infoList[index].m_size += (2+1) + (40 + 1);
	}
	// update
	m_msg.setDataShort(data.m_pressure, m_infoList[index].m_rawGpsPressure);
	m_msg.setDataByte(data.m_pressureAge, m_infoList[index].m_rawGpsPressureAge);

	// lon,lat,height,hacc,vacc,veln,vele,veld
	int32_t* bareln = (int32_t*)&data.m_itow;
	for (uint16_t i=0; i<10;++i)
		m_msg.setDataLong(bareln[i],m_infoList[index].m_rawGpsGpsData + (4*i));

	// gpsAge
	m_msg.setDataByte(data.m_gpsAge,  m_infoList[index].m_rawGpsGpsAge);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Temperature component of a data item.
double Packet::getTemp(const uint16_t index) const
{
	if (containsTemp(index))
		return m_msg.getDataFPValue(m_formatList[index].m_outputSettings, m_infoList[index].m_temp);

	return 0.0;
}
bool Packet::containsTemp(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_temp == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateTemp(const double& temp, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_temp == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_temp = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + ds);
		m_infoList[index].m_size += ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, temp, m_infoList[index].m_temp);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Accelerometer component of a data item.
CmtVector Packet::getCalAcc(const uint16_t index) const
{
	CmtVector buffer;
	if (containsCalAcc(index))
		m_msg.getDataFPValue(&buffer.m_data[0], m_formatList[index].m_outputSettings, m_infoList[index].m_calAcc, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsCalAcc(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_calAcc == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateCalAcc(const CmtVector& vec, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_calAcc == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_calAcc = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, &vec.m_data[0], m_infoList[index].m_calAcc, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Gyroscope component of a data item.
CmtVector Packet::getCalGyr(const uint16_t index) const
{
	CmtVector buffer;
	if (containsCalGyr(index))
		m_msg.getDataFPValue(&buffer.m_data[0], m_formatList[index].m_outputSettings, m_infoList[index].m_calGyr, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsCalGyr(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_calGyr == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateCalGyr(const CmtVector& vec, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_calGyr == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_calGyr = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, &vec.m_data[0], m_infoList[index].m_calGyr, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Magnetometer component of a data item.
CmtVector Packet::getCalMag(const uint16_t index) const
{
	CmtVector buffer;
	if (containsCalMag(index))
		m_msg.getDataFPValue(&buffer.m_data[0], m_formatList[index].m_outputSettings, m_infoList[index].m_calMag, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsCalMag(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_calMag == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateCalMag(const CmtVector& vec, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_calMag == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_calMag = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, &vec.m_data[0], m_infoList[index].m_calMag, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Calibrated Accelerometer component of a data item.
CmtCalData Packet::getCalData(const uint16_t index) const
{
	CmtCalData buffer;
	double* bare = (double*) &buffer;
	if (containsCalData(index))
		m_msg.getDataFPValue(bare, m_formatList[index].m_outputSettings, m_infoList[index].m_calData, 9);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsCalData(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_calData == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateCalData(const CmtCalData& data, const uint16_t index)
{
	const uint16_t numValues = 9;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_calData == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_msg.m_autoUpdateChecksum = false;
		m_infoList[index].m_calData = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_calAcc = m_infoList[index].m_calData;
		m_infoList[index].m_calGyr = m_infoList[index].m_calData + 3*ds;
		m_infoList[index].m_calMag = m_infoList[index].m_calData + 6*ds;
		m_infoList[index].m_size += numValues*ds;
	}
	// update

	double* bare = (double*) &data;
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, bare, m_infoList[index].m_calData, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as a Quaternion.
CmtQuat Packet::getOriQuat(const uint16_t index) const
{
	CmtQuat buffer;
	if (containsOriQuat(index))
		m_msg.getDataFPValue(&buffer.m_data[0], m_formatList[index].m_outputSettings, m_infoList[index].m_oriQuat, 4);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}

bool Packet::containsOriQuat(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_oriQuat == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateOriQuat(const CmtQuat& data, const uint16_t index)
{
	const uint16_t numValues = 4;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_oriQuat == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_oriQuat = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, &data.m_data[0], m_infoList[index].m_oriQuat, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as CmtEuler angles.
CmtEuler Packet::getOriEuler(const uint16_t index) const
{
	CmtEuler buffer;
	if (containsOriEuler(index))
	{
		uint16_t ds = getFPValueSize(index);
		buffer.m_roll  = m_msg.getDataFPValue(m_formatList[index].m_outputSettings, m_infoList[index].m_oriEul);
		buffer.m_pitch = m_msg.getDataFPValue(m_formatList[index].m_outputSettings, m_infoList[index].m_oriEul + ds);
		buffer.m_yaw   = m_msg.getDataFPValue(m_formatList[index].m_outputSettings, m_infoList[index].m_oriEul + 2*ds);
	}
	return buffer;
}
bool Packet::containsOriEuler(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_oriEul == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateOriEuler(const CmtEuler& data, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_oriEul == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_oriEul = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, data.m_roll,  m_infoList[index].m_oriEul);
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, data.m_pitch, m_infoList[index].m_oriEul + ds);
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, data.m_yaw,   m_infoList[index].m_oriEul + 2*ds);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Orientation component of a data item as an Orientation Matrix.
CmtMatrix Packet::getOriMatrix(const uint16_t index) const
{
	CmtMatrix buffer;
	uint16_t k = 0;
	if (containsOriMatrix(index))
	{
		uint16_t ds = getFPValueSize(index);
		for (int32_t i=0;i<3;++i)
			for (int32_t j=0;j<3;++j, k+=ds)
				buffer.m_data[i][j] = m_msg.getDataFPValue(m_formatList[index].m_outputSettings, m_infoList[index].m_oriMat+k);
	}
	else
		memset(&buffer, 0, sizeof(buffer));

	return buffer;
}
bool Packet::containsOriMatrix(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_oriMat == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateOriMatrix(const CmtMatrix& data, const uint16_t index)
{
	const uint16_t numValues = 9;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_oriMat == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_oriMat = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	uint16_t k = 0;
	for (int32_t i=0;i<3;++i)
		for (int32_t j=0;j<3;++j, k+=ds)
			m_msg.setDataFPValue(m_formatList[index].m_outputSettings, data.m_data[i][j], m_infoList[index].m_oriMat+k);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the AnalogIn 1 component of a data item.
CmtAnalogInData Packet::getAnalogIn1(const uint16_t index) const
{
	CmtAnalogInData buffer;
	if (containsAnalogIn1(index))
		buffer.m_data = m_msg.getDataShort(m_infoList[index].m_analogIn1);

	return buffer;
}
bool Packet::containsAnalogIn1(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_analogIn1 == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateAnalogIn1(const CmtAnalogInData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_analogIn1 == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_analogIn1 = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(data.m_data, m_infoList[index].m_analogIn1);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the AnalogIn 2 component of a data item.
CmtAnalogInData Packet::getAnalogIn2(const uint16_t index) const
{
	CmtAnalogInData buffer;
	if (containsAnalogIn2(index))
		buffer.m_data = m_msg.getDataShort(m_infoList[index].m_analogIn2);

	return buffer;
}
bool Packet::containsAnalogIn2(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_analogIn2 == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateAnalogIn2(const CmtAnalogInData& data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_analogIn2 == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_analogIn2 = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(data.m_data, m_infoList[index].m_analogIn2);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Position LLA component of a data item.
CmtVector Packet::getPositionLLA(const uint16_t index) const
{
	CmtVector buffer;
	if (containsPositionLLA(index))
		m_msg.getDataFPValue(&buffer.m_data[0], m_formatList[index].m_outputSettings, m_infoList[index].m_posLLA, 3);
	else
		memset(&buffer, 0, sizeof(buffer));
	return buffer;
}
bool Packet::containsPositionLLA(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_posLLA == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updatePositionLLA(const CmtVector& data, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_posLLA == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_posLLA = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, &data.m_data[0], m_infoList[index].m_posLLA, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Velocity NED component of a data item.
CmtVector Packet::getVelocity(const uint16_t index) const
{
	CmtVector buffer;
	if (containsVelocity(index))
		m_msg.getDataFPValue(&buffer.m_data[0], m_formatList[index].m_outputSettings, m_infoList[index].m_velNEDorNWU, 3);
	else
		memset(&buffer, 0, sizeof(buffer));

	return buffer;
}
bool Packet::containsVelocity(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_velNEDorNWU == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateVelocity(const CmtVector& data, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_velNEDorNWU == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_velNEDorNWU = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, &data.m_data[0], m_infoList[index].m_velNEDorNWU, numValues);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Status component of a data item.
uint8_t Packet::getStatus(const uint16_t index) const
{
	if (containsStatus(index))
		return m_msg.getDataByte(m_infoList[index].m_status);
	return 0;
}
bool Packet::containsStatus(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_status == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateStatus(const uint8_t data, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;

	if (m_infoList[index].m_status == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_status = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 1);
		m_infoList[index].m_size += 1;
	}
	// update
	m_msg.setDataByte(data,m_infoList[index].m_status);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the Sample Counter component of the packet.
uint16_t Packet::getSampleCounter(const uint16_t index) const
{
	if (!containsSampleCounter(index))
		return 0;
	return m_msg.getDataShort(m_infoList[index].m_sc);
}
bool Packet::containsSampleCounter(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_sc == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateSampleCounter(const uint16_t counter, const uint16_t index)
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_sc == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add
		m_infoList[index].m_sc = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + 2);
		m_infoList[index].m_size += 2;
	}
	// update
	m_msg.setDataShort(counter, m_infoList[index].m_sc);
	return true;
}
TimeStamp Packet::getRtc(const uint16_t index) const
{
	MRPT_UNUSED_PARAM(index);
	return m_rtc;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the XKF-3 Acc G component of the packet.
CmtVector Packet::getAccG(const uint16_t index) const
{
	CmtVector buffer;
	if (containsAccG(index))
		m_msg.getDataFPValue(&buffer.m_data[0], m_formatList[index].m_outputSettings, m_infoList[index].m_acc_g, 3);
	else
		memset(&buffer, 0, sizeof(buffer));

	return buffer;
}
bool Packet::containsAccG(const uint16_t index) const
{
	if (getDataSize(index) == 0)
		return false;
	if (m_infoList[index].m_acc_g == CMT_DATA_ITEM_NOT_AVAILABLE)
		return false;
	return true;
}
bool Packet::updateAccG(const CmtVector& g, const uint16_t index)
{
	const uint16_t numValues = 3;
	if (getDataSize(index) == 0)
		return false;

	uint16_t ds = getFPValueSize(index);

	if (m_infoList[index].m_acc_g == CMT_DATA_ITEM_NOT_AVAILABLE)
	{
		// add space
		m_infoList[index].m_acc_g = m_msg.getDataSize();
		m_msg.resizeData(m_msg.getDataSize() + numValues*ds);
		m_infoList[index].m_size += numValues*ds;
	}
	// update
	m_msg.setDataFPValue(m_formatList[index].m_outputSettings, &g.m_data[0], m_infoList[index].m_acc_g, numValues);
	return true;
}

Packet::Packet(const Packet& pack)
{
	PACKETLOG("Create new packet from Packet %p\n",&pack);

	m_itemCount = 0;
	m_formatList = NULL;
	m_infoList = NULL;
	*this = pack;
}

void Packet::operator = (const Packet& pack)
{
	PACKETLOG("Copy packet from Packet %p\n",this);

	if (m_itemCount != pack.m_itemCount)
	{
		LSTCHKDELNUL(m_formatList);
		m_itemCount = pack.m_itemCount;
		m_formatList = new CmtDataFormat[m_itemCount];
	}
	for (uint16_t i = 0; i < m_itemCount; ++i)
		m_formatList[i] = pack.m_formatList[i];
	LSTCHKDELNUL(m_infoList);
	m_toa = pack.m_toa;
	m_rtc = pack.m_rtc;
	m_msg = pack.m_msg;
	m_xm = pack.m_xm;
}

#ifdef _CMT_DLL_EXPORT
void Packet::interpolate(const Packet& pa, const Packet& pb, const double f)
{
	pa.getDataSize();
	pb.getDataSize();

	*this = pb;

	if (m_itemCount != pa.m_itemCount)
		return;

	getDataSize();
	CmtShortVector va, vb, vc;
	CmtVector fa, fb, fc;
	CmtQuat qf;
	CmtEuler ea, eb, ec;
	CmtMatrix mf;

	Quaternion qa, qb, qc;
	Matrix3x3 m;

	double g = 1.0-f;
	double da, db, dc;
	uint16_t sa, sb, sc;

	// walk over all items and interpolate all data items
	for (uint16_t i = 0; i < m_itemCount; ++i)
	{
		//// raw data
		if (pa.m_infoList[i].m_rawAcc != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_rawAcc != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			va = pa.getRawAcc(i);		vb = pb.getRawAcc(i);
			vc.m_data[0] = (uint16_t) (g*(double)va.m_data[0] + f*(double)vb.m_data[0]);
			vc.m_data[1] = (uint16_t) (g*(double)va.m_data[1] + f*(double)vb.m_data[1]);
			vc.m_data[2] = (uint16_t) (g*(double)va.m_data[2] + f*(double)vb.m_data[2]);
			updateRawAcc(vc,i);
		}

		if (pa.m_infoList[i].m_rawGyr != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_rawGyr != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			va = pa.getRawGyr(i);		vb = pb.getRawGyr(i);
			vc.m_data[0] = (uint16_t) (g*(double)va.m_data[0] + f*(double)vb.m_data[0]);
			vc.m_data[1] = (uint16_t) (g*(double)va.m_data[1] + f*(double)vb.m_data[1]);
			vc.m_data[2] = (uint16_t) (g*(double)va.m_data[2] + f*(double)vb.m_data[2]);
			updateRawGyr(vc,i);
		}

		if (pa.m_infoList[i].m_rawMag != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_rawMag != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			va = pa.getRawMag(i);		vb = pb.getRawMag(i);
			vc.m_data[0] = (uint16_t) (g*(double)va.m_data[0] + f*(double)vb.m_data[0]);
			vc.m_data[1] = (uint16_t) (g*(double)va.m_data[1] + f*(double)vb.m_data[1]);
			vc.m_data[2] = (uint16_t) (g*(double)va.m_data[2] + f*(double)vb.m_data[2]);
			updateRawMag(vc,i);
		}

		if (pa.m_infoList[i].m_rawTemp != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_rawTemp != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			da = (double) pa.getRawTemp(i);		db = (double) pb.getRawTemp(i);
			dc = g*da + f*db;
			updateRawTemp((uint16_t) dc,i);
		}

		//// calibrated data
		if (pa.m_infoList[i].m_calAcc != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_calAcc != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			fa = pa.getCalAcc(i);		fb = pb.getCalAcc(i);
			fc.m_data[0] = (g*(double)fa.m_data[0] + f*(double)fb.m_data[0]);
			fc.m_data[1] = (g*(double)fa.m_data[1] + f*(double)fb.m_data[1]);
			fc.m_data[2] = (g*(double)fa.m_data[2] + f*(double)fb.m_data[2]);
			updateCalAcc(fc,i);
		}

		if (pa.m_infoList[i].m_calGyr != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_calGyr != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			fa = pa.getCalGyr(i);		fb = pb.getCalGyr(i);
			fc.m_data[0] = (g*(double)fa.m_data[0] + f*(double)fb.m_data[0]);
			fc.m_data[1] = (g*(double)fa.m_data[1] + f*(double)fb.m_data[1]);
			fc.m_data[2] = (g*(double)fa.m_data[2] + f*(double)fb.m_data[2]);
			updateCalGyr(fc,i);
		}

		if (pa.m_infoList[i].m_calMag != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_calMag != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			fa = pa.getCalMag(i);		fb = pb.getCalMag(i);
			fc.m_data[0] = (g*(double)fa.m_data[0] + f*(double)fb.m_data[0]);
			fc.m_data[1] = (g*(double)fa.m_data[1] + f*(double)fb.m_data[1]);
			fc.m_data[2] = (g*(double)fa.m_data[2] + f*(double)fb.m_data[2]);
			updateCalMag(fc,i);
		}

		//// orientations
		if (pa.m_infoList[i].m_oriQuat != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_oriQuat != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			qf = pa.getOriQuat(i);
			qa.isArray(qf.m_data);

			qf = pb.getOriQuat(i);
			qb.isArray(qf.m_data);

			qc.isSlerp(qa,qb,f);
			qc.setArray(qf.m_data);

			updateOriQuat(qf,i);
		}

		if (pa.m_infoList[i].m_oriEul != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_oriEul != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			ea = pa.getOriEuler(i);		eb = pb.getOriEuler(i);
			ec.m_roll  = (g*(double)ea.m_roll  + f*(double)eb.m_roll);
			ec.m_pitch = (g*(double)ea.m_pitch + f*(double)eb.m_pitch);
			ec.m_yaw   = (g*(double)ea.m_yaw   + f*(double)eb.m_yaw);
			updateOriEuler(ec,i);
		}

		if (pa.m_infoList[i].m_oriMat != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_oriMat != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			mf = pa.getOriMatrix(i);
			m.isArray(&mf.m_data[0][0]);
			qa.isRmat(m);

			mf = pb.getOriMatrix(i);
			m.isArray(&mf.m_data[0][0]);
			qb.isRmat(m);

			qc.isSlerp(qa,qb,f);
			m.isQuat(qc);

			m.setArray(&mf.m_data[0][0]);
			updateOriMatrix(mf,i);
		}

		//// other
		if (pa.m_infoList[i].m_sc != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_sc != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			sa = pa.getSampleCounter(i);	sb = pb.getSampleCounter(i);
			sc = sb - sa;
			dc = f*(double) sc;
			sc = sa + (uint16_t) doubleToLong(dc);
			updateSampleCounter(sc,i);
		}

		if (pa.m_infoList[i].m_acc_g != CMT_DATA_ITEM_NOT_AVAILABLE && pb.m_infoList[i].m_acc_g != CMT_DATA_ITEM_NOT_AVAILABLE)
		{
			fa = pa.getAccG(i);		fb = pb.getAccG(i);
			fc.m_data[0] = (g*(double)fa.m_data[0] + f*(double)fb.m_data[0]);
			fc.m_data[1] = (g*(double)fa.m_data[1] + f*(double)fb.m_data[1]);
			fc.m_data[2] = (g*(double)fa.m_data[2] + f*(double)fb.m_data[2]);
			updateAccG(fc,i);
		}
	}
	// some remaining stuff
	// TOA remains equal to pb TOA
	m_rtc = (TimeStamp) (g*(double) pa.m_rtc + f*(double) pb.m_rtc);
}
#endif

} // end of xsens namespace
