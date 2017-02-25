/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef LEGACYDATAPACKET_H
#define LEGACYDATAPACKET_H

#include <xsens/pstdint.h>
#include <xsens/xstypedefs.h>
#include <xsens/xsdataformat.h>
#include <xsens/xsdeviceid.h>
#include <xsens/xstimestamp.h>
#include <xsens/xsmessage.h>

//! Indicates that a data item is not available in the packet
#define XS_DATA_ITEM_NOT_AVAILABLE		65535

struct XsVector;
struct PacketInfo;
struct XsUShortVector;
struct XsScrData;
struct XsCalibratedData;
struct XsGpsPvtData;
struct XsPressure;
struct MtwSdiData;
struct XsQuaternion;
struct XsMatrix;
struct XsEuler;
struct XsAnalogInData;
struct XsUtcTime;

//! \brief An MT timestamp (sample count)
typedef uint16_t XsMtTimeStamp;

struct PacketFixedData;

struct LegacyDataPacket
{
public:

	LegacyDataPacket();
	LegacyDataPacket(uint16_t count, bool xbus);
	LegacyDataPacket(const LegacyDataPacket& pack);
	~LegacyDataPacket();

	const LegacyDataPacket& operator = (const LegacyDataPacket& pack);

	uint16_t	itemCount(void) const;
	void		setItemCount(uint16_t count);

	XsTimeStamp	timeOfArrival(void) const;
	void		setTimeOfArrival(XsTimeStamp timeOfArrival);

	XsTimeStamp	rtc(void) const;
	void		setRtc(XsTimeStamp rtc);

	int64_t		largePacketCounter(void) const;
	void		setLargePacketCounter(int64_t sc);

	XsMessage	message(void) const;
	void		setMessage(const XsMessage& message);
	XsMessage	originalMessage(void) const;

	PacketInfo	packetInfo(int32_t index) const;

	bool setDataFormat(const XsDataFormat& format, int32_t index = 0);
	bool setDataFormat(XsOutputMode outputMode, XsOutputSettings outputSettings, int32_t index = 0);
	XsDataFormat dataFormat(int32_t index = 0) const;

	void setXbusSystem(bool xbus, bool convert = false);
	bool isXbusSystem(void) const;

	int32_t findDeviceId(XsDeviceId dev) const;
	void setDeviceId(XsDeviceId deviceId, int32_t index);
	XsDeviceId deviceId(int32_t index) const;

	uint16_t frameCounter() const;

	XsSize dataSize(int32_t index=0) const;

	void updateInfoList();

	//PacketFixedData takePacketFixedData();
	//void putPacketFixedData(const PacketFixedData& data);

	uint16_t getFPValueSize(int32_t index) const;

	XsUShortVector rawAcceleration(int32_t index=0) const;
	bool containsRawAcceleration(int32_t index=0) const;
	bool setRawAcceleration(const XsUShortVector& vec, int32_t index=0);
	/*! \brief Return the Raw Gyroscope component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Raw Gyroscope component of a data item.
	*/
	XsUShortVector rawGyroscopeData(int32_t index=0) const;
		//! Check if data item contains Raw Gyroscope data
	bool containsRawGyroscopeData(int32_t index=0) const;
		//! Add/update Raw Gyroscope data for the item
	bool setRawGyroscopeData(const XsUShortVector& vec, int32_t index=0);
	/*! \brief Return the Raw Magnetometer component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Raw Magnetometer component of a data item.
	*/
	XsUShortVector rawMagneticField(int32_t index=0) const;
		//! Check if data item contains Raw Magnetometer data
	bool containsRawMagneticField(int32_t index=0) const;
		//! Add/update Raw Magnetometer data for the item
	bool setRawMagneticField(const XsUShortVector& vec, int32_t index=0);
	/*! \brief Return the Raw Temperature component of a data item.
		\param index The index of the item of which the data should be returned.
		\param channel The Temperature channel to return
		\returns The Raw Temperature component of a data item.
	*/
	uint16_t rawTemperature(int32_t index=0, int channel = 0) const;
		//! Check if data item contains Raw Temperature data
	bool containsRawTemperature(int32_t index=0, int channel = 0) const;
		//! Add/update Raw Temperature data for the item
	bool setRawTemperature(uint16_t temp, int32_t index=0, int channel = 0);
		//! Returns the number of available Raw Temperature channels
	int rawTemperatureChannelCount(int32_t index = 0) const;
	/*! \brief Return the Raw Data component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Raw Data component of a data item.
	*/
	XsScrData rawData(int32_t index=0) const;
		//! Check if data item contains Raw Data
	bool containsRawData(int32_t index=0) const;
		//! Add/update Raw Data for the item
	bool setRawData(const XsScrData& data, int32_t index=0);
	/*! \brief Return the Gps PVT data component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Gps PVT data component of a data item.
	*/
	XsGpsPvtData gpsPvtData(int32_t index=0) const;
	//! Check if data item contains Gps PVT Data
	bool containsGpsPvtData(int32_t index=0) const;
	//! Add/update Gps PVT Data for the item
	bool setGpsPvtData(const XsGpsPvtData& data, int32_t index=0);

	XsPressure pressure(int32_t index=0) const;
	bool containsPressure(int32_t index=0) const;
	bool setPressure(const XsPressure& data, int32_t index=0);

	MtwSdiData mtwSdiData(int32_t index=0) const;
	bool containsMtwSdiData(int32_t index=0) const;
	bool setMtwSdiData(const MtwSdiData& data, int32_t index=0);

	/*! \brief Return the Temperature component of a data item.
		\param index The index of the item of which the data should be returned.
		\param channel The Temperature channel to return
		\returns The Temperature component of a data item.
	*/
	double temperature(int32_t index=0, int channel = 0) const;
	//! Check if data item contains Temperature data
	bool containsTemperature(int32_t index=0, int channel = 0) const;
	//! Add/update Calibrated Accelerometer data for the item
	bool setTemperature(const double& temp, int32_t index=0, int channel = 0);
	//! Returns the number of temperature channels
	int temperatureChannelCount(int32_t index = 0) const;

	/*! \brief Return the Calibrated Accelerometer component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Calibrated Accelerometer component of a data item.
	*/
	XsVector calibratedAcceleration(int32_t index=0) const;
		//! Check if data item contains Calibrated Accelerometer data
	bool containsCalibratedAcceleration(int32_t index=0) const;
		//! Add/update Calibrated Accelerometer data for the item
	bool setCalibratedAcceleration(const XsVector& vec, int32_t index=0);

	/*! \brief Return the Calibrated Gyroscope component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Calibrated Gyroscope component of a data item.
	*/
	XsVector calibratedGyroscopeData(int32_t index=0) const;
		//! Check if data item contains Calibrated Gyroscope data
	bool containsCalibratedGyroscopeData(int32_t index=0) const;
		//! Add/update Calibrated Gyroscope data for the item
	bool setCalibratedGyroscopeData(const XsVector& vec, int32_t index=0);

	/*! \brief Return the Calibrated Magnetometer component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Calibrated Magnetometer component of a data item.
	*/
	XsVector calibratedMagneticField(int32_t index=0) const;
		//! Check if data item contains Calibrated Magnetometer data
	bool containsCalibratedMagneticField(int32_t index=0) const;
		//! Add/update Calibrated Magnetometer data for the item
	bool setCalibratedMagneticField(const XsVector& vec, int32_t index=0);

	XsCalibratedData calibratedData(int32_t index=0) const;
	bool containsCalibratedData(int32_t index=0) const;
	bool setCalibratedData(const XsCalibratedData& data, int32_t index=0);
	/*! \brief Return the Orientation component of a data item as a Quaternion.
		\param index The index of the item of which the data should be returned.
		\returns The Orientation component of a data item as a Quaternion.
	*/
	XsQuaternion orientationQuaternion(int32_t index=0) const;
		//! Check if data item contains Quaternion Orientation data
	bool containsOrientationQuaternion(int32_t index=0) const;
		//! Add/update Quaternion Orientation data for the item
	bool setOrientationQuaternion(const XsQuaternion& data, int32_t index=0);
	/*! \brief Return the Orientation component of a data item as Euler angles.
		\param index The index of the item of which the data should be returned.
		\returns The Orientation component of a data item as Euler angles.
	*/
	XsEuler orientationEuler(int32_t index=0) const;
		//! Check if data item contains Euler Orientation data
	bool containsOrientationEuler(int32_t index=0) const;
		//! Add/update Euler Orientation data for the item
	bool setOrientationEuler(const XsEuler& data, int32_t index=0);
	/*! \brief Return the Orientation component of a data item as an Orientation Matrix.
		\param index The index of the item of which the data should be returned.
		\returns The Orientation component of a data item as an Orientation Matrix.
	*/
	XsMatrix orientationMatrix(int32_t index=0) const;
		//! Check if data item contains Matrix Orientation data
	bool containsOrientationMatrix(int32_t index=0) const;
		//! Add/update Matrix Orientation data for the item
	bool setOrientationMatrix(const XsMatrix& data, int32_t index=0);

		//! Check if data item contains Orientation Data of any kind
	bool containsOrientation(int32_t index=0) const;

	/*! \brief Return the AnalogIn 1 component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The AnalogIn 1 component of a data item.
	*/
	XsAnalogInData analogIn1Data(int32_t index=0) const;
	//! Check if data item contains AnalogIn 1
	bool containsAnalogIn1Data(int32_t index=0) const;
	//! Add/update AnalogIn 1 for the item
	bool setAnalogIn1Data(const XsAnalogInData& data, int32_t index=0);

	/*! \brief Return the AnalogIn 2 component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The AnalogIn 2 component of a data item.
	*/
	XsAnalogInData analogIn2Data(int32_t index=0) const;
	//! Check if data item contains AnalogIn 2
	bool containsAnalogIn2Data(int32_t index=0) const;
	//! Add/update AnalogIn 2 for the item
	bool setAnalogIn2Data(const XsAnalogInData& data, int32_t index=0);

	/*! \brief Return the Position Lat Lon Alt component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Position Lat Lon Alt component of a data item.
	*/
	XsVector positionLLA(int32_t index=0) const;
	//! Check if data item contains Position Lat Lon Alt
	bool containsPositionLLA(int32_t index=0) const;
	//! Add/update Position Lat Lon Alt for the item
	bool setPositionLLA(const XsVector& data, int32_t index=0);

	/*! \brief Return the Velocity component of a data item.
		\param index The index of the item of which the data should be returned.
		\returns The Velocity component of a data item.
	*/
	XsVector velocity(int32_t index=0) const;
	//! Check if data item contains Velocity
	bool containsVelocity(int32_t index=0) const;
	//! Add/update Velocity for the item
	bool setVelocity(const XsVector& data, int32_t index=0);

	/*! \brief Return the Status component of a data item.
		\param index The index of the item of which the data should be returned.
		\param[out] outIsDetailed pointer to boolean. After call, the boolean is true if the status is detailed
		\returns The Status component of a data item.
	*/
	uint32_t status(int32_t index, bool* outIsDetailed) const;
	//! Check if data item contains Status
	bool containsStatus(int32_t index=0) const;
	//! Check if data item contains detailed Status information
	bool containsDetailedStatus(int32_t index=0) const;
	//! Add/update Status information for the item
	bool setStatus(const uint32_t data, int32_t index=0);

	/*! \brief Return the Sample Counter component of the packet.
		\param index The index of the item of which the data should be returned. (ignored)
		\returns The Sample Counter component of the packet.
	*/
	uint16_t packetCounter(int32_t index=0) const;
		//! Check if data item contains Sample Counter
	bool containsPacketCounter(int32_t index=0) const;
		//! Add/update Sample Counter for all items
	bool setPacketCounter(const uint16_t counter, int32_t index=0);

	/*! \brief Return the UTC Time component of the packet.
		\param index The index of the item of which the data should be returned. (ignored)
		\returns The UTC Time component of the packet.
	*/
	XsUtcTime utcTime(int32_t index=0) const;
		//! Check if data item contains UTC Time
	bool containsUtcTime(int32_t index=0) const;
		//! Add/update UTC Time for all items
	bool setUtcTime(const XsUtcTime& data, int32_t index=0);

	/*! \brief Return the Acc-G component of the packet.
		\param index The index of the item of which the data should be returned.
		\returns The Acc-G component of the packet.
	*/
	XsVector freeAcceleration(int32_t index=0) const;
		//! Check if data item contains XKF-3 Acc-G data
	bool containsFreeAcceleration(int32_t index=0) const;
		//! Add/update XKF-3 Acc-G data for the item
	bool setFreeAcceleration(const XsVector& g, int32_t index=0);

	/*! \brief Return the synhcronization time recorded by the station.
		\param channelID The synchronization channel number, where the output signal is connected.
		\returns The synhcronization time recorded by the station.
	*/
	XsTimeStamp triggerIndication(int channelID) const;
		//! Check if data item contains indication time on the given line
	bool containsTriggerIndication(int channelID = 0) const;
		//! Add/update trigger indication time for the item to the given line
	bool setTriggerIndication(int channelID, const XsTimeStamp &t);

	/*! \brief Returns a const pointer to the contained fixed data, mostly used for debugging */
	inline const PacketFixedData* fixedData() const { return m_fixedData; }

private:
	PacketFixedData*		m_fixedData;			//!< Fixed packet data
	mutable XsDeviceId		m_lastFoundId;	//!< Last found deviceId, speeds up searches
	mutable uint16_t		m_lastFoundIndex;		//!< Index of last found deviceId, speeds up searches
	XsMessage			m_msg;						//!< The message
	XsTimeStamp		m_rtc;					//!< Sample time in ms, based on the sample counter
	XsTimeStamp		m_toa;					//!< Time of arrival
	XsTimeStamp		m_packetId;					//!< 64 bit sample counter
	XsTimeStamp		m_triggerIn1;			//!< Trigger indication on line 1	\todo remove this
	XsTimeStamp		m_triggerIn2;			//!< Trigger indication on line 2	\todo remove this
};

#endif	// file guard
