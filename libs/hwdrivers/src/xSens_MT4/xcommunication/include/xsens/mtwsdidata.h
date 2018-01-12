/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef MTWSDIDATA_H
#define MTWSDIDATA_H

#include <xsens/pstdint.h>
#include <xsens/xsvector3.h>
#include <xsens/xsquaternion.h>
#include <xsens/xsdeviceid.h>

struct XsRange;

struct MtwSdiData
{
	/** The ID of the device that generated the data */
	XsDeviceId m_deviceId;
	/** Indicates if the time sync is in order (unused) */
	uint8_t m_timeSync;
	/** The first frame number of the SDI interval. The time of the interval is
	 * [first, last) */
	uint16_t m_firstFrameNumber;
	/** The last frame number of the SDI interval. The time of the interval is
	 * [first, last) */
	uint16_t m_lastFrameNumber;
	/** The gyroscope bias used during the SDI interval */
	XsVector3 m_currentBias;
	/** The orientation increment (delta Q) over the interval */
	XsQuaternion m_orientationIncrement;
	/** The velocity increment (delta V) over the interval */
	XsVector3 m_velocityIncrement;
	/** reserved */
	bool m_aidingData;
	/** The barometer value during the interval */
	double m_barometer;
	/** The magnetometer values during the interval */
	XsVector3 m_magnetoMeter;
	/** The Received Signal Strength Indication (RSSI) of the message */
	int8_t m_rssi;

	MtwSdiData();
	MtwSdiData(const MtwSdiData& other);
	~MtwSdiData();
	const MtwSdiData& operator=(const MtwSdiData& other);

	bool empty() const;
	bool containsAidingData() const;
	XsQuaternion orientationIncrement() const;
	XsVector velocityIncrement() const;
	double pressure() const;
	XsVector magneticField() const;
	XsVector currentBias() const;
	XsRange frameRange() const;
	double rssi() const;
};

#endif  // file guard
