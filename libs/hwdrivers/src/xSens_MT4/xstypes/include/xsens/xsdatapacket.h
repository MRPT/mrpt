/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSDATAPACKET_H
#define XSDATAPACKET_H

#include "xstypedefs.h"
#include "pstdint.h"
#include "xsmessage.h"
#include "xstimestamp.h"
#include "xsdataidentifier.h"
#include "xsushortvector.h"
#include "xsscrdata.h"
#include "xscalibrateddata.h"
#include "xsgpspvtdata.h"
#include "xspressure.h"
#include "xssdidata.h"
#include "xsvector.h"
#include "xsquaternion.h"
#include "xsmatrix.h"
#include "xseuler.h"
#include "xsanalogindata.h"
#include "xsutctime.h"
#include "xsrawgpsdop.h"
#include "xsrawgpssol.h"
#include "xsrawgpssvinfo.h"
#include "xsrawgpstimeutc.h"
#include "xsdeviceid.h"
#include "xsrange.h"
#include "xstriggerindicationdata.h"

struct XsDataPacket;
#ifdef __cplusplus
extern "C" 
{
#else
typedef struct XsDataPacket XsDataPacket;
#define XSDATAPACKET_INITIALIZER	{ XSMESSAGE_INITIALIZER, XSMESSAGE_INITIALIZER, XSDEVICEID_INITIALIZER, XDI_None, -1, 0, 0, XSTIMESTAMP_INITIALIZER, XSTIMESTAMP_INITIALIZER }
#endif

XSTYPES_DLL_API void XsDataPacket_construct(XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_destruct(XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_copy(XsDataPacket* copy, XsDataPacket const* src);
XSTYPES_DLL_API void XsDataPacket_swap(XsDataPacket* a, XsDataPacket* b);
XSTYPES_DLL_API int XsDataPacket_empty(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_itemOffsetExact(const XsDataPacket* thisPtr, XsDataIdentifier id); 
XSTYPES_DLL_API int XsDataPacket_itemOffsetLoose(const XsDataPacket* thisPtr, XsDataIdentifier id);
XSTYPES_DLL_API int XsDataPacket_itemOffsetMasked(const XsDataPacket* thisPtr, XsDataIdentifier id, XsDataIdentifier mask);
XSTYPES_DLL_API void XsDataPacket_setMessage(XsDataPacket* thisPtr, const XsMessage* msg);
XSTYPES_DLL_API XsMessage* XsDataPacket_originalMessage(const XsDataPacket* thisPtr, XsMessage* returnVal);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_dataFormat(const XsDataPacket* thisPtr, XsDataIdentifier id);
XSTYPES_DLL_API uint8_t XsDataPacket_getFPValueSize(XsDataIdentifier id);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawAcceleration(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawAcceleration(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawAcceleration(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawGyroscopeData(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGyroscopeData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGyroscopeData(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawGyroscopeTemperatureData(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGyroscopeTemperatureData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGyroscopeTemperatureData(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawMagneticField(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawMagneticField(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawMagneticField(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API uint16_t XsDataPacket_rawTemperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsRawTemperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawTemperature(XsDataPacket* thisPtr, uint16_t temp);
XSTYPES_DLL_API XsScrData* XsDataPacket_rawData(const XsDataPacket* thisPtr, XsScrData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawData(XsDataPacket* thisPtr, const XsScrData* data);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedAcceleration(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedAcceleration(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedGyroscopeData(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedGyroscopeData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedGyroscopeData(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedMagneticField(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedMagneticField(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedMagneticField(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsCalibratedData* XsDataPacket_calibratedData(const XsDataPacket* thisPtr, XsCalibratedData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedData(XsDataPacket* thisPtr, const XsCalibratedData* data);
XSTYPES_DLL_API XsQuaternion* XsDataPacket_orientationQuaternion(const XsDataPacket* thisPtr, XsQuaternion* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API void XsDataPacket_setOrientationQuaternion(XsDataPacket* thisPtr, const XsQuaternion* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API XsEuler* XsDataPacket_orientationEuler(const XsDataPacket* thisPtr, XsEuler* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API void XsDataPacket_setOrientationEuler(XsDataPacket* thisPtr, const XsEuler* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API XsMatrix* XsDataPacket_orientationMatrix(const XsDataPacket* thisPtr, XsMatrix* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API void XsDataPacket_setOrientationMatrix(XsDataPacket* thisPtr, const XsMatrix* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API int XsDataPacket_containsOrientation(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_orientationIdentifier(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_coordinateSystemOrientation(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsSdiData* XsDataPacket_sdiData(const XsDataPacket* thisPtr, XsSdiData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsSdiData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setSdiData(XsDataPacket* thisPtr, const XsSdiData* data);
XSTYPES_DLL_API uint32_t XsDataPacket_status(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsStatus(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsStatusByte(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsDetailedStatus(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setStatus(XsDataPacket* thisPtr, uint32_t data);
XSTYPES_DLL_API void XsDataPacket_setStatusByte(XsDataPacket* thisPtr, uint8_t data);
XSTYPES_DLL_API uint8_t XsDataPacket_packetCounter8(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsPacketCounter8(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setPacketCounter8(XsDataPacket* thisPtr, uint8_t counter);
XSTYPES_DLL_API uint16_t XsDataPacket_packetCounter(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsPacketCounter(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setPacketCounter(XsDataPacket* thisPtr, uint16_t counter);
XSTYPES_DLL_API uint32_t XsDataPacket_sampleTimeFine(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsSampleTimeFine(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setSampleTimeFine(XsDataPacket* thisPtr, uint32_t counter);
XSTYPES_DLL_API uint32_t XsDataPacket_sampleTimeCoarse(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsSampleTimeCoarse(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setSampleTimeCoarse(XsDataPacket* thisPtr, uint32_t counter);
XSTYPES_DLL_API uint64_t XsDataPacket_sampleTime64(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsSampleTime64(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setSampleTime64(XsDataPacket* thisPtr, uint64_t counter);
XSTYPES_DLL_API XsVector* XsDataPacket_freeAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsFreeAcceleration(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setFreeAcceleration(XsDataPacket* thisPtr, const XsVector* g);
XSTYPES_DLL_API double XsDataPacket_temperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsTemperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setTemperature(XsDataPacket* thisPtr, double temp);
XSTYPES_DLL_API XsGpsPvtData* XsDataPacket_gpsPvtData(const XsDataPacket* thisPtr, XsGpsPvtData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsGpsPvtData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setGpsPvtData(XsDataPacket* thisPtr, const XsGpsPvtData* data);
XSTYPES_DLL_API int XsDataPacket_containsPressure(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsPressure* XsDataPacket_pressure(const XsDataPacket* thisPtr, XsPressure* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsPressureAge(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setPressure(XsDataPacket* thisPtr, const XsPressure* data);
XSTYPES_DLL_API XsAnalogInData* XsDataPacket_analogIn1Data(const XsDataPacket* thisPtr, XsAnalogInData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsAnalogIn1Data(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAnalogIn1Data(XsDataPacket* thisPtr, const XsAnalogInData* data);
XSTYPES_DLL_API XsAnalogInData* XsDataPacket_analogIn2Data(const XsDataPacket* thisPtr, XsAnalogInData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsAnalogIn2Data(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAnalogIn2Data(XsDataPacket* thisPtr, const XsAnalogInData* data);
XSTYPES_DLL_API XsVector* XsDataPacket_positionLLA(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsPositionLLA(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setPositionLLA(XsDataPacket* thisPtr, const XsVector* data);
XSTYPES_DLL_API XsVector* XsDataPacket_latitudeLongitude(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsLatitudeLongitude(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setLatitudeLongitude(XsDataPacket* thisPtr, const XsVector* data);
XSTYPES_DLL_API double XsDataPacket_altitude(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsAltitude(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAltitude(XsDataPacket* thisPtr, double data);
XSTYPES_DLL_API XsVector* XsDataPacket_velocity(const XsDataPacket* thisPtr, XsVector* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API int XsDataPacket_containsVelocity(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setVelocity(XsDataPacket* thisPtr, const XsVector* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_velocityIdentifier(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_coordinateSystemVelocity(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsUtcTime* XsDataPacket_utcTime(const XsDataPacket* thisPtr, XsUtcTime* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsUtcTime(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setUtcTime(XsDataPacket* thisPtr, const XsUtcTime* data);
XSTYPES_DLL_API XsRange* XsDataPacket_frameRange(const XsDataPacket* thisPtr, XsRange* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsFrameRange(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setFrameRange(XsDataPacket* thisPtr, const XsRange* r);
XSTYPES_DLL_API int XsDataPacket_rssi(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsRssi(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRssi(XsDataPacket* thisPtr, int r);
XSTYPES_DLL_API XsRawGpsDop* XsDataPacket_rawGpsDop(const XsDataPacket* thisPtr, XsRawGpsDop* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGpsDop(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsRawGpsSol* XsDataPacket_rawGpsSol(const XsDataPacket* thisPtr, XsRawGpsSol* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGpsSol(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsRawGpsTimeUtc* XsDataPacket_rawGpsTimeUtc(const XsDataPacket* thisPtr, XsRawGpsTimeUtc* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGpsTimeUtc(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsRawGpsSvInfo* XsDataPacket_rawGpsSvInfo(const XsDataPacket* thisPtr, XsRawGpsSvInfo* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGpsSvInfo(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsDataPacket* XsDataPacket_append(XsDataPacket* thisPtr, const XsDataPacket* other);
XSTYPES_DLL_API void XsDataPacket_setTriggerIndication(XsDataPacket* thisPtr, XsDataIdentifier triggerId, XsTriggerIndicationData const * triggerIndicationData);
XSTYPES_DLL_API XsTriggerIndicationData* XsDataPacket_triggerIndication(XsDataPacket const * thisPtr, XsDataIdentifier triggerId, XsTriggerIndicationData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsTriggerIndication(XsDataPacket const * thisPtr, XsDataIdentifier triggerId);

#ifdef __cplusplus
} // extern "C"
#endif

struct XsDataPacket {
#ifdef __cplusplus
	/*! \brief Default constructor, initializes empty data packet or from the supplied \a msg
		\param msg Either 0 to create an empty object or a pointer to a valid %XsMessage containing
		MTData2 data.
	*/
	explicit XsDataPacket(const XsMessage* msg = 0)
		: m_lastFoundId(XDI_None)
		, m_lastFoundOffset(-1)
		, m_itemCount(0)
		, m_originalMessageLength(0)
		, m_toa(0)
		, m_packetId(0)
	{
		XsMessage_setMessageId(&m_msg, XMID_InvalidMessage);
		XsMessage_setMessageId(&m_legacyMsg, XMID_InvalidMessage);

		if (msg)
			XsDataPacket_setMessage(this, msg);
	}

	/*! \brief Copy constructor
		\param pack The packet to copy from
	*/
	XsDataPacket(const XsDataPacket& pack)
		: m_lastFoundId(XDI_None)
		, m_lastFoundOffset(-1)
		, m_itemCount(0)
		, m_originalMessageLength(0)
		, m_toa(0)
		, m_packetId(0)
	{
		*this = pack;
	}

	//! \copydoc XsDataPacket_destruct
	~XsDataPacket()
	{
		XsDataPacket_destruct(this);
	}

	/*! \brief Assignment operator
		\param other The packet to copy from
		\returns A reference to this %XsDataPacket
		\sa XsDataPacket_copy
	*/
	const XsDataPacket& operator = (const XsDataPacket& other)
	{
		if (this != &other)
			XsDataPacket_copy(this, &other);
		return *this;
	}

	/*! \brief \copybrief XsDataPacket_empty */
	inline bool empty(void) const
	{
		return 0 != XsDataPacket_empty(this);
	}

	/*! \copydoc XsDataPacket_itemOffsetExact */
	inline int itemOffsetExact(XsDataIdentifier id) const
	{
		return XsDataPacket_itemOffsetExact(this, id);
	}

	/*! \copydoc XsDataPacket_itemOffsetLoose */
	inline int itemOffsetLoose(XsDataIdentifier id) const
	{
		return XsDataPacket_itemOffsetLoose(this, id);
	}

	/*! \brief Return the id that was last found successfully by itemOffsetLoose or itemOffsetExact
		\details When the last search didn't find anything, XDI_None will be returned.
		\returns The id that was last found successfully by itemOffsetLoose or itemOffsetExact or XDI_None
		\sa itemOffsetExact \sa itemOffsetLoose \sa lastFoundOffset
	*/
	inline XsDataIdentifier lastFoundId() const
	{
		return m_lastFoundId;
	}

	/*! \brief Return the offset that was last returned by itemOffsetLoose or itemOffsetExact
		\details When the last search didn't find anything, -1 will be returned.
		\returns The offset that was last returned by itemOffsetLoose or itemOffsetExact
		\sa itemOffsetExact \sa itemOffsetLoose \sa lastFoundId
	*/
	inline int lastFoundOffset() const
	{
		return m_lastFoundOffset;
	}

	//! \brief Return the device ID associated with the data packet
	inline XsDeviceId deviceId() const
	{
		return m_deviceId;
	}

	//! \brief Return the number of data items in the packet
	inline uint16_t itemCount() const
	{
		return m_itemCount;
	}

	//! \copydoc XsDataPacket_setMessage
	inline void setMessage(const XsMessage& msg)
	{
		XsDataPacket_setMessage(this, &msg);
	}

	/*! \brief Returns a const reference to the message that contains the data packet
	*/
	inline const XsMessage& XSNOCOMEXPORT message() const
	{
		return m_msg;
	}

	/*! \brief Returns a reference to the message that contains the data packet
		\returns A reference to the message that contains the data packet
		\note Modifying this message directly can cause %XsDataPacket to break
	*/
	inline XsMessage& XSNOCOMEXPORT message()
	{
		return m_msg;	//lint !e1536
	}

	//! \brief Set the device ID associated with this data packet
	inline void setDeviceId(const XsDeviceId id)
	{
		m_deviceId = id;
	}

	/*! \brief Returns a copy the original message of the data packet
		\details This returns the original message that was last set with setMessage,
		or in the constructor. Note that if existing data was updated, the original data may have been
		overwritten, only added data will not be returned in this message.
		When the packet was constructed from a legacy message, the legacy message will be returned.
		\sa XsDataPacket_originalMessage
		\returns An XsMessage containing a copy of the original message
	*/
	inline XsMessage XSNOCOMEXPORT originalMessage(void) const
	{
		XsMessage returnVal;
		return *XsDataPacket_originalMessage(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_dataFormat */
	inline XsDataIdentifier dataFormat(XsDataIdentifier id) const
	{
		return XsDataPacket_dataFormat(this, id);
	}

	/*! \copydoc XsDataPacket_getFPValueSize */
	inline static uint8_t getFPValueSize(XsDataIdentifier id)
	{
		return XsDataPacket_getFPValueSize(id);
	}

	/*! \brief \copybrief XsDataPacket_rawAcceleration */
	inline XsUShortVector rawAcceleration(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawAcceleration(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawAcceleration */
	inline bool containsRawAcceleration(void) const
	{
		return 0 != XsDataPacket_containsRawAcceleration(this);
	}

	/*! \copydoc XsDataPacket_setRawAcceleration */
	inline void setRawAcceleration(const XsUShortVector& vec)
	{
		XsDataPacket_setRawAcceleration(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeData */
	inline XsUShortVector rawGyroscopeData(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawGyroscopeData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawGyroscopeData */
	inline bool containsRawGyroscopeData(void) const
	{
		return 0 != XsDataPacket_containsRawGyroscopeData(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGyroscopeData */
	inline void setRawGyroscopeData(const XsUShortVector& vec)
	{
		XsDataPacket_setRawGyroscopeData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeTemperatureData */
	inline XsUShortVector rawGyroscopeTemperatureData(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawGyroscopeTemperatureData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawGyroscopeTemperatureData */
	inline bool containsRawGyroscopeTemperatureData(void) const
	{
		return 0 != XsDataPacket_containsRawGyroscopeTemperatureData(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGyroscopeTemperatureData */
	inline void setRawGyroscopeTemperatureData(const XsUShortVector& vec)
	{
		XsDataPacket_setRawGyroscopeTemperatureData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawMagneticField */
	inline XsUShortVector rawMagneticField(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawMagneticField(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawMagneticField */
	inline bool containsRawMagneticField(void) const
	{
		return 0 != XsDataPacket_containsRawMagneticField(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawMagneticField */
	inline void setRawMagneticField(const XsUShortVector& vec)
	{
		XsDataPacket_setRawMagneticField(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawTemperature */
	inline uint16_t rawTemperature(void) const
	{
		return XsDataPacket_rawTemperature(this);
	}

	/*! \copydoc XsDataPacket_containsRawTemperature */
	inline bool containsRawTemperature(void) const
	{
		return 0 != XsDataPacket_containsRawTemperature(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawTemperature */
	inline void setRawTemperature(uint16_t temp)
	{
		XsDataPacket_setRawTemperature(this, temp);
	}

	/*! \brief \copybrief XsDataPacket_rawData */
	inline XsScrData rawData(void) const
	{
		XsScrData returnVal;
		return *XsDataPacket_rawData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawData */
	inline bool containsRawData(void) const
	{
		return 0 != XsDataPacket_containsRawData(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawData */
	inline void setRawData(const XsScrData& data)
	{
		XsDataPacket_setRawData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_calibratedAcceleration */
	inline XsVector calibratedAcceleration(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedAcceleration(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedAcceleration */
	inline bool containsCalibratedAcceleration(void) const
	{
		return 0 != XsDataPacket_containsCalibratedAcceleration(this);
	}

	/*! \brief \copybrief XsDataPacket_setCalibratedAcceleration */
	inline void setCalibratedAcceleration(const XsVector& vec)
	{
		XsDataPacket_setCalibratedAcceleration(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedGyroscopeData */
	inline XsVector calibratedGyroscopeData(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedGyroscopeData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedGyroscopeData */
	inline bool containsCalibratedGyroscopeData(void) const
	{
		return 0 != XsDataPacket_containsCalibratedGyroscopeData(this);
	}

	/*! \brief \copybrief XsDataPacket_setCalibratedGyroscopeData */
	inline void setCalibratedGyroscopeData(const XsVector& vec)
	{
		XsDataPacket_setCalibratedGyroscopeData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedMagneticField */
	inline XsVector calibratedMagneticField(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedMagneticField(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedMagneticField */
	inline bool containsCalibratedMagneticField(void) const
	{
		return 0 != XsDataPacket_containsCalibratedMagneticField(this);
	}

	/*! \brief \copybrief XsDataPacket_setCalibratedMagneticField */
	inline void setCalibratedMagneticField(const XsVector& vec)
	{
		XsDataPacket_setCalibratedMagneticField(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedData */
	inline XsCalibratedData calibratedData(void) const
	{
		XsCalibratedData returnVal;
		return *XsDataPacket_calibratedData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedData */
	inline bool containsCalibratedData(void) const
	{
		return 0 != XsDataPacket_containsCalibratedData(this);
	}

	/*! \brief \copybrief XsDataPacket_setCalibratedData */
	inline void setCalibratedData(const XsCalibratedData& data)
	{
		XsDataPacket_setCalibratedData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_orientationQuaternion */
	inline XsQuaternion orientationQuaternion(XsDataIdentifier coordinateSystem) const
	{
		XsQuaternion returnVal;
		return *XsDataPacket_orientationQuaternion(this, &returnVal, coordinateSystem);
	}

	/*! \brief returns the orientation as a quaternion with the current coordinate system*/
	inline XsQuaternion orientationQuaternion() const
	{
		XsQuaternion returnVal;
		return *XsDataPacket_orientationQuaternion(this, &returnVal, coordinateSystemOrientation());
	}

	/*! \brief \copybrief XsDataPacket_setOrientationQuaternion */
	inline void setOrientationQuaternion(const XsQuaternion& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationQuaternion(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_orientationEuler */
	inline XsEuler orientationEuler(XsDataIdentifier coordinateSystem) const
	{
		XsEuler returnVal;
		return *XsDataPacket_orientationEuler(this, &returnVal, coordinateSystem);
	}

	/*! \brief returns the orientation as an XsEuler with the current coordinate system*/
	inline XsEuler orientationEuler() const
	{
		XsEuler returnVal;
		return *XsDataPacket_orientationEuler(this, &returnVal, coordinateSystemOrientation());
	}

	/*! \brief \copybrief XsDataPacket_setOrientationEuler */
	inline void setOrientationEuler(const XsEuler& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationEuler(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_orientationMatrix */
	inline XsMatrix orientationMatrix(XsDataIdentifier coordinateSystem) const
	{
		XsMatrix returnVal;
		return *XsDataPacket_orientationMatrix(this, &returnVal, coordinateSystem);
	}

	/*! \brief returns the orientation as an orientation matrix with the current coordinate system*/
	inline XsMatrix orientationMatrix() const
	{
		XsMatrix returnVal;
		return *XsDataPacket_orientationMatrix(this, &returnVal, coordinateSystemOrientation());
	}

	/*! \brief \copybrief XsDataPacket_setOrientationMatrix */
	inline void setOrientationMatrix(const XsMatrix& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationMatrix(this, &data, coordinateSystem);
	}

	/*! \copydoc XsDataPacket_containsOrientation */
	inline bool containsOrientation(void) const
	{
		return 0 != XsDataPacket_containsOrientation(this);
	}

	/*! \copydoc XsDataPacket_orientationIdentifier */
	inline XsDataIdentifier orientationIdentifier() const
	{
		return XsDataPacket_orientationIdentifier(this);
	}

	/*! \copydoc XsDataPacket_coordinateSystemOrientation */
	inline XsDataIdentifier coordinateSystemOrientation() const
	{
		return XsDataPacket_coordinateSystemOrientation(this);
	}

	/*! \brief \copybrief XsDataPacket_sdiData */
	inline XsSdiData sdiData(void) const
	{
		XsSdiData returnVal;
		return *XsDataPacket_sdiData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsSdiData */
	inline bool containsSdiData(void) const
	{
		return 0 != XsDataPacket_containsSdiData(this);
	}

	/*! \copydoc XsDataPacket_setSdiData */
	inline void setSdiData(const XsSdiData& data)
	{
		XsDataPacket_setSdiData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_status */
	inline uint32_t status(void) const
	{
		return XsDataPacket_status(this);
	}

	/*! \copydoc XsDataPacket_containsStatus */
	inline bool containsStatus(void) const
	{
		return 0 != XsDataPacket_containsStatus(this);
	}

	/*! \copydoc XsDataPacket_containsStatusByte */
	inline bool containsStatusByte(void) const
	{
		return 0 != XsDataPacket_containsStatusByte(this);
	}

	/*! \copydoc XsDataPacket_containsDetailedStatus */
	inline bool containsDetailedStatus(void) const
	{
		return 0 != XsDataPacket_containsDetailedStatus(this);
	}

	/*! \brief \copybrief XsDataPacket_setStatus */
	inline void setStatus(const uint32_t data)
	{
		XsDataPacket_setStatus(this, data);
	}

	/*! \brief \copybrief XsDataPacket_setStatusByte */
	inline void setStatusByte(const uint8_t data)
	{
		XsDataPacket_setStatusByte(this, data);
	}

	/*! \brief \copybrief XsDataPacket_packetCounter8 */
	inline uint8_t packetCounter8(void) const
	{
		return XsDataPacket_packetCounter8(this);
	}

	/*! \copydoc XsDataPacket_containsPacketCounter8 */
	inline bool containsPacketCounter8(void) const
	{
		return 0 != XsDataPacket_containsPacketCounter8(this);
	}

	/*! \brief \copybrief XsDataPacket_setPacketCounter8 */
	inline void setPacketCounter8(uint8_t counter)
	{
		XsDataPacket_setPacketCounter8(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_packetCounter */
	inline uint16_t packetCounter(void) const
	{
		return XsDataPacket_packetCounter(this);
	}

	/*! \copydoc XsDataPacket_containsPacketCounter */
	inline bool containsPacketCounter(void) const
	{
		return 0 != XsDataPacket_containsPacketCounter(this);
	}

	/*! \brief \copybrief XsDataPacket_setPacketCounter */
	inline void setPacketCounter(uint16_t counter)
	{
		XsDataPacket_setPacketCounter(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTimeFine */
	inline uint32_t sampleTimeFine(void) const
	{
		return XsDataPacket_sampleTimeFine(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTimeFine */
	inline bool containsSampleTimeFine(void) const
	{
		return 0 != XsDataPacket_containsSampleTimeFine(this);
	}

	/*! \brief \copybrief XsDataPacket_setSampleTimeFine */
	inline void setSampleTimeFine(uint32_t counter)
	{
		XsDataPacket_setSampleTimeFine(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTimeCoarse */
	inline uint32_t sampleTimeCoarse(void) const
	{
		return XsDataPacket_sampleTimeCoarse(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTimeCoarse */
	inline bool containsSampleTimeCoarse(void) const
	{
		return 0 != XsDataPacket_containsSampleTimeCoarse(this);
	}

	/*! \brief \copybrief XsDataPacket_setSampleTimeCoarse */
	inline void setSampleTimeCoarse(uint32_t counter)
	{
		XsDataPacket_setSampleTimeCoarse(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTime64 */
	inline uint64_t sampleTime64(void) const
	{
		return XsDataPacket_sampleTime64(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTime64 */
	inline bool containsSampleTime64(void) const
	{
		return 0 != XsDataPacket_containsSampleTime64(this);
	}

	/*! \brief \copybrief XsDataPacket_setSampleTime64 */
	inline void setSampleTime64(uint64_t counter)
	{
		XsDataPacket_setSampleTime64(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_freeAcceleration */
	inline XsVector freeAcceleration(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_freeAcceleration(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsFreeAcceleration */
	inline bool containsFreeAcceleration(void) const
	{
		return 0 != XsDataPacket_containsFreeAcceleration(this);
	}

	/*! \brief \copybrief XsDataPacket_setFreeAcceleration */
	inline void setFreeAcceleration(const XsVector& g)
	{
		XsDataPacket_setFreeAcceleration(this, &g);
	}

	/*! \brief \copybrief XsDataPacket_temperature */
	inline double temperature(void) const
	{
		return XsDataPacket_temperature(this);
	}

	/*! \copydoc XsDataPacket_containsTemperature */
	inline bool containsTemperature(void) const
	{
		return 0 != XsDataPacket_containsTemperature(this);
	}

	/*! \brief \copybrief XsDataPacket_setTemperature */
	inline void setTemperature(double temp)
	{
		XsDataPacket_setTemperature(this, temp);
	}

	/*! \brief \copybrief XsDataPacket_gpsPvtData */
	inline XsGpsPvtData gpsPvtData(void) const
	{
		XsGpsPvtData returnVal;
		return *XsDataPacket_gpsPvtData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsGpsPvtData */
	inline bool containsGpsPvtData(void) const
	{
		return 0 != XsDataPacket_containsGpsPvtData(this);
	}

	/*! \copydoc XsDataPacket_setGpsPvtData */
	inline void setGpsPvtData(const XsGpsPvtData& data)
	{
		XsDataPacket_setGpsPvtData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_pressure */
	inline XsPressure pressure(void) const
	{
		XsPressure returnVal;
		return *XsDataPacket_pressure(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsPressure */
	inline bool containsPressure(void) const
	{
		return 0 != XsDataPacket_containsPressure(this);
	}

	/*! \copydoc XsDataPacket_containsPressureAge */
	inline bool containsPressureAge(void) const
	{
		return 0 != XsDataPacket_containsPressureAge(this);
	}

	/*! \brief \copybrief XsDataPacket_setPressure */
	inline void setPressure(const XsPressure& data)
	{
		XsDataPacket_setPressure(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_analogIn1Data */
	inline XsAnalogInData analogIn1Data(void) const
	{
		XsAnalogInData returnVal;
		return *XsDataPacket_analogIn1Data(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsAnalogIn1Data */
	inline bool containsAnalogIn1Data(void) const
	{
		return 0 != XsDataPacket_containsAnalogIn1Data(this);
	}

	/*! \brief \copybrief XsDataPacket_setAnalogIn1Data */
	inline void setAnalogIn1Data(const XsAnalogInData& data)
	{
		XsDataPacket_setAnalogIn1Data(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_analogIn2Data */
	inline XsAnalogInData analogIn2Data(void) const
	{
		XsAnalogInData returnVal;
		return *XsDataPacket_analogIn2Data(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsAnalogIn2Data */
	inline bool containsAnalogIn2Data(void) const
	{
		return 0 != XsDataPacket_containsAnalogIn2Data(this);
	}

	/*! \brief \copybrief XsDataPacket_setAnalogIn2Data */
	inline void setAnalogIn2Data(const XsAnalogInData& data)
	{
		XsDataPacket_setAnalogIn2Data(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_positionLLA */
	inline XsVector positionLLA(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_positionLLA(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsPositionLLA */
	inline bool containsPositionLLA(void) const
	{
		return 0 != XsDataPacket_containsPositionLLA(this);
	}

	/*! \copydoc XsDataPacket_setPositionLLA */
	inline void setPositionLLA(const XsVector& data)
	{
		XsDataPacket_setPositionLLA(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_latitudeLongitude */
	inline XsVector latitudeLongitude(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_latitudeLongitude(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsLatitudeLongitude */
	inline bool containsLatitudeLongitude(void) const
	{
		return 0 != XsDataPacket_containsLatitudeLongitude(this);
	}

	/*! \copydoc XsDataPacket_setLatitudeLongitude */
	inline void setLatitudeLongitude(const XsVector& data)
	{
		XsDataPacket_setLatitudeLongitude(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_altitude */
	inline double altitude(void) const
	{
		return XsDataPacket_altitude(this);
	}

	/*! \copydoc XsDataPacket_containsAltitude */
	inline bool containsAltitude(void) const
	{
		return 0 != XsDataPacket_containsAltitude(this);
	}

	/*! \copydoc XsDataPacket_setAltitude */
	inline void setAltitude(double data)
	{
		XsDataPacket_setAltitude(this, data);
	}

	/*! \brief \copybrief XsDataPacket_velocity */
	inline XsVector velocity(XsDataIdentifier coordinateSystem) const
	{
		XsVector returnVal;
		return *XsDataPacket_velocity(this, &returnVal, coordinateSystem);
	}
	
	/*! \brief returns the velocity with the current coordinate system*/
	inline XsVector velocity(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_velocity(this, &returnVal, coordinateSystemVelocity());
	}

	/*! \copydoc XsDataPacket_containsVelocity */
	inline bool containsVelocity(void) const
	{
		return 0 != XsDataPacket_containsVelocity(this);
	}

	/*! \brief \copybrief XsDataPacket_setVelocity */
	inline void setVelocity(const XsVector& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setVelocity(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_velocityIdentifier */
	inline XsDataIdentifier velocityIdentifier() const
	{
		return XsDataPacket_velocityIdentifier(this);
	}

	/*! \copydoc XsDataPacket_coordinateSystemVelocity */
	inline XsDataIdentifier coordinateSystemVelocity() const
	{
		return XsDataPacket_coordinateSystemVelocity(this);
	}

	/*! \brief \copybrief XsDataPacket_utcTime */
	inline XsUtcTime utcTime(void) const
	{
		XsUtcTime returnVal;
		return *XsDataPacket_utcTime(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsUtcTime */
	inline bool containsUtcTime(void) const
	{
		return 0 != XsDataPacket_containsUtcTime(this);
	}

	/*! \brief \copybrief XsDataPacket_setUtcTime */
	inline void setUtcTime(const XsUtcTime& data)
	{
		XsDataPacket_setUtcTime(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_frameRange */
	inline XsRange frameRange() const
	{
		XsRange returnVal;
		return *XsDataPacket_frameRange(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsFrameRange */
	inline bool containsFrameRange() const
	{
		return 0 != XsDataPacket_containsFrameRange(this);
	}

	/*! \copydoc XsDataPacket_setFrameRange */
	inline void setFrameRange(const XsRange& r)
	{
		XsDataPacket_setFrameRange(this, &r);
	}

	/*! \brief \copybrief XsDataPacket_rssi */
	inline int rssi() const
	{
		return XsDataPacket_rssi(this);
	}

	/*! \copydoc XsDataPacket_containsRssi */
	inline bool containsRssi() const
	{
		return 0 != XsDataPacket_containsRssi(this);
	}

	/*! \copydoc XsDataPacket_setRssi */
	inline void setRssi(int r)
	{
		XsDataPacket_setRssi(this, r);
	}

	/*! \brief \copybrief XsDataPacket_rawGpsDop */
	inline XsRawGpsDop rawGpsDop(void) const
	{
		XsRawGpsDop returnVal;
		return *XsDataPacket_rawGpsDop(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsDop */
	inline bool containsRawGpsDop(void) const
	{
		return 0 != XsDataPacket_containsRawGpsDop(this);
	}

	/*! \brief \copybrief XsDataPacket_rawGpsSol */
	inline XsRawGpsSol rawGpsSol(void) const
	{
		XsRawGpsSol returnVal = XsRawGpsSol();
		return *XsDataPacket_rawGpsSol(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsSol */
	inline bool containsRawGpsSol(void) const
	{
		return 0 != XsDataPacket_containsRawGpsSol(this);
	}

	/*! \brief \copybrief XsDataPacket_rawGpsTimeUtc */
	inline XsRawGpsTimeUtc rawGpsTimeUtc(void) const
	{
		XsRawGpsTimeUtc returnVal;
		return *XsDataPacket_rawGpsTimeUtc(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsTimeUtc */
	inline bool containsRawGpsTimeUtc(void) const
	{
		return 0 != XsDataPacket_containsRawGpsTimeUtc(this);
	}

	/*! \brief \copybrief XsDataPacket_rawGpsSvInfo */
	inline XsRawGpsSvInfo rawGpsSvInfo(void) const
	{
		XsRawGpsSvInfo returnVal;
		return *XsDataPacket_rawGpsSvInfo(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGpsSvInfo */
	inline bool containsRawGpsSvInfo(void) const
	{
		return 0 != XsDataPacket_containsRawGpsSvInfo(this);
	}

	/*! \copydoc XsDataPacket_append */
	inline XsDataPacket& append(const XsDataPacket& other)
	{
		return *XsDataPacket_append(this, &other);
	}

	/*! \private \brief Set the time of arrival of the data packet */
	inline void setTimeOfArrival(XsTimeStamp t)
	{
		m_toa = t;
	}

	/*! \brief Return the time of arrival of the data packet. Only valid for live streams. The behaviour for file streams is undefined and may change in the future. */
	inline XsTimeStamp timeOfArrival() const
	{
		return m_toa;
	}

	/*! \private \brief Set the packet ID of the data packet*/
	inline void setPacketId(XsTimeStamp t)
	{
		m_packetId = t;
	}

	/*! \brief Return the ID of the packet.
		\details This ID is based on, depending on availability: (1) packet counter (2) sample time (3) arrival order
		\returns The ID of the packet.
	*/
	inline XsTimeStamp packetId() const
	{
		return m_packetId;
	}

	/*! \copydoc XsDataPacket_setTriggerIndication */
	void setTriggerIndication(XsDataIdentifier triggerId, XsTriggerIndicationData const & triggerIndicationData)
	{
		XsDataPacket_setTriggerIndication(this, triggerId, &triggerIndicationData);
	}

/*! \brief Returns the trigger indication data of a packet
    \details
	If the packet does not contain the requested data, the return val struct will be set to all zeroes	
	\param[in] triggerId The trigger data identifier to add data for (e.g. XDI_TriggerIn1 or XDI_TriggerIn2) 
	\returns the trigger indication data of a packet
*/
	XsTriggerIndicationData triggerIndication(XsDataIdentifier triggerId)
	{
		XsTriggerIndicationData returnVal;
		return *XsDataPacket_triggerIndication(this, triggerId, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsTriggerIndication */
	int containsTriggerIndication(XsDataIdentifier triggerId)
	{
		return XsDataPacket_containsTriggerIndication(this, triggerId);
	}

protected:
	/*! \privatesection */
#endif // __cplusplus

	XsMessage			m_msg;						//!< The message that contains the data
	XsMessage			m_legacyMsg;				//!< Optional legacy MtData message as received (for logging the received data only)
	XsDeviceId			m_deviceId;					//!< The device Id to which the message belongs
	XsDataIdentifier	m_lastFoundId;				//!< Last found data identifer, speeds up searches
	int					m_lastFoundOffset;			//!< Offset of last found data identifier, speeds up searches
	uint16_t			m_itemCount;				//!< The number of data items in the message
	uint16_t			m_originalMessageLength;	//!< Length of the original message payload
	XsTimeStamp			m_toa;						//!< Time of arrival (live packets only)
	XsTimeStamp			m_packetId;					//!< 64 bit packet id, based on, depending on availability: (1) packet counter (2) sample time (3) arrival order
};

#endif // file guard
