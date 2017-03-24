/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef PACKETFIXEDDATA_H
#define PACKETFIXEDDATA_H

#include <xsens/xsscrdata.h>
#include <xsens/xsdeviceid.h>

#ifndef NOT_FOR_PUBLIC_RELEASE
struct XsControl;	// required by DLL for supporting advanced features
#endif

struct XsDataFormat;

//! Indicates that a data item is not available in the packet
#define XS_DATA_ITEM_NOT_AVAILABLE		65535

/*! \brief Contains offset information about data in the packet
	\details All items are initialized to \a XS_DATA_ITEM_NOT_AVAILABLE and set to their proper
	value by LegacyDataPacket::updateInfoList and/or the LegacyDataPacket update functions.
*/
struct PacketInfo {
	uint16_t m_offset;		//!< Offset of first data byte (whatever it is)
	uint16_t m_rawData;		//!< Offset of raw data
	uint16_t m_rawAcc;		//!< Offset of raw acceleration data
	uint16_t m_rawGyr;		//!< Offset of raw gyroscope data
	uint16_t m_rawMag;		//!< Offset of raw magnetometer data
	uint16_t m_rawTemp[XS_MAX_TEMPERATURE_CHANNELS];	//!< Offset of raw temperature data. Note that usually only the first item is used.
	uint16_t m_temp[XS_MAX_TEMPERATURE_CHANNELS];		//!< Offset of calibrated temperature data. Note that usually only the first item is used.
	uint16_t m_calData;		//!< Offset of calibrated data
	uint16_t m_calAcc;		//!< Offset of calibrated acceleration data
	uint16_t m_calGyr;		//!< Offset of calibrated gyroscope data
	uint16_t m_calMag;		//!< Offset of calibrated magnetometer data
	uint16_t m_oriQuat;		//!< Offset of orientation in quaternion format
	uint16_t m_oriEul;		//!< Offset of orientation in euler format
	uint16_t m_oriMat;		//!< Offset of orientation in matrix format
	uint16_t m_analogIn1;	//!< Offset of analog in channel 1 data
	uint16_t m_analogIn2;	//!< Offset of analog in channel 2 data
	uint16_t m_posLLA;		//!< Offset of Latitude-Longitude-Altitude position data
	uint16_t m_velNEDorNWU;	//!< Offset of velocity data
	uint16_t m_status;		//!< Offset of status data
	uint16_t m_detailedStatus;	//!< Offset of detailed status data
	uint16_t m_sc;			//!< Offset of sample counter
	uint16_t m_utcTime;		//!< Offset of UTC time data
	uint16_t m_utcNano;		//!< Offset of nanosecond part of UTC time
	uint16_t m_utcYear;		//!< Offset of year part of UTC time
	uint16_t m_utcMonth;	//!< Offset of month part of UTC time
	uint16_t m_utcDay;		//!< Offset of day part of UTC time
	uint16_t m_utcHour;		//!< Offset of hour part of UTC time
	uint16_t m_utcMinute;	//!< Offset of minute part of UTC time
	uint16_t m_utcSecond;	//!< Offset of second part of UTC time
	uint16_t m_utcValid;	//!< Offset of validity part of UTC time
	uint16_t m_acc_g;		//!< Offset of acceleration in global frame
	uint16_t m_gpsPvtData;	//!< Offset of GPS & pressure data
	uint16_t m_gpsPvtPressure;		//!< Offset of pressure data
	uint16_t m_gpsPvtPressureAge;	//!< Offset of pressure age
	uint16_t m_gpsPvtGpsData;	//!< Offset of raw GPS data
	uint16_t m_gpsPvtItow;		//!< Offset of raw GPS ITOW (Integer Time Of Week) data
	uint16_t m_gpsPvtLatitude;	//!< Offset of raw GPS latitude data
	uint16_t m_gpsPvtLongitude;	//!< Offset of raw GPS longitude data
	uint16_t m_gpsPvtHeight;	//!< Offset of raw GPS height data
	uint16_t m_gpsPvtVeln;		//!< Offset of raw GPS northward velocity data
	uint16_t m_gpsPvtVele;		//!< Offset of raw GPS eastward velocity data
	uint16_t m_gpsPvtVeld;		//!< Offset of raw GPS dowanward velocity data
	uint16_t m_gpsPvtHacc;		//!< Offset of raw GPS horizontal accuracy estimate data
	uint16_t m_gpsPvtVacc;		//!< Offset of raw GPS vertical accuracy estimate data
	uint16_t m_gpsPvtSacc;		//!< Offset of raw GPS speed accuracy estimate data
	uint16_t m_gpsPvtGpsAge;	//!< Offset of raw GPS age
	uint16_t m_mtwSdiData;				//!< Offset of MTw SDI data
	uint16_t m_wClientId;				//!< Offset of MTw SDI client ID data
	uint16_t m_wTimeSync;				//!< Offset of MTw SDI time sync data
	uint16_t m_wFirstFrameNumber;		//!< Offset of MTw SDI first frame number in interval data
	uint16_t m_wLastFrameNumber;		//!< Offset of MTw SDI last frame number in interval data
	uint16_t m_wCurrentBias;			//!< Offset of MTw SDI gyroscope bias data
	uint16_t m_wOrientationIncrement;	//!< Offset of MTw SDI orientation increment data
	uint16_t m_wVelocityIncrement;		//!< Offset of MTw SDI velocity increment data
	uint16_t m_wAidingData;				//!< Offset of MTw SDI aiding data
	uint16_t m_wBaroMeter;				//!< Offset of MTw SDI barometer data
	uint16_t m_wMagnetoMeter;			//!< Offset of MTw SDI magnetometer data
	uint16_t m_wRssi;					//!< Offset of MTw SDI RSSI data
	uint16_t m_size;			//!< Total size of the data
	uint16_t m_doubleBoundary;	//!< Boundary where the original data format is ignored and values are stored in double precision

	/*! \brief Default contructor, sets all values to XS_DATA_ITEM_NOT_AVAILABLE
	*/
	PacketInfo() { memset(this, 0xFF, sizeof(*this)); }
};

//! A structure containing fixed packet data, which should not change during a measurement for the same device
struct PacketFixedData
{
	PacketFixedData();
	PacketFixedData(uint16_t count);
	PacketFixedData(const PacketFixedData& pack);
	~PacketFixedData();

	void operator = (const PacketFixedData& data);

	PacketInfo*		m_infoList;		//!< Contains information about data in the packet and the format of that data
	XsDataFormat*	m_formatList;	//!< A list of the formats of the data items
	XsDeviceId*		m_idList;		//!< A list of the device-ids in this packet
	bool			m_xm;			//!< Indicates that xbus-formatting is used
	uint16_t		m_itemCount;	//!< The number of data items in the message
};

#endif	// file guard
