
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

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
#include "xspressure.h"
#include "xssdidata.h"
#include "xsvector.h"
#include "xsquaternion.h"
#include "xsmatrix.h"
#include "xseuler.h"
#include "xsanalogindata.h"
#include "xstimeinfo.h"
#include "xsrawgnsspvtdata.h"
#include "xsrawgnsssatinfo.h"
#include "xsdeviceid.h"
#include "xsrange.h"
#include "xstriggerindicationdata.h"
#include "xssnapshot.h"
#include "xsglovesnapshot.h"
#include "xsglovedata.h"

#ifndef XSNOEXPORT
#define XSNOEXPORT
#endif

//AUTO namespace xstypes {
struct XsDataPacket;
//AUTO }
struct XSNOEXPORT DataPacketPrivate;

#ifdef __cplusplus
extern "C"
{
#endif
#ifndef __cplusplus
typedef struct XsDataPacket XsDataPacket;
//#define XSDATAPACKET_INITIALIZER	{ 0, 0, XSDEVICEID_INITIALIZER, -1 } //Use XsDataPacket_construct in all cases because of dynamic initialization
#endif

XSTYPES_DLL_API void XsDataPacket_construct(XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_copyConstruct(XsDataPacket* thisPtr, XsDataPacket const* src);
XSTYPES_DLL_API void XsDataPacket_destruct(XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_clear(XsDataPacket* thisPtr, XsDataIdentifier id);
XSTYPES_DLL_API void XsDataPacket_copy(XsDataPacket* copy, XsDataPacket const* src);
XSTYPES_DLL_API void XsDataPacket_swap(XsDataPacket* thisPtr, XsDataPacket* other);
XSTYPES_DLL_API int XsDataPacket_empty(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_itemCount(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setMessage(XsDataPacket* thisPtr, const XsMessage* msg);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_dataFormat(const XsDataPacket* thisPtr, XsDataIdentifier id);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawAcceleration(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API XsVector* XsDataPacket_rawAccelerationConverted(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawAcceleration(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawAcceleration(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawGyroscopeData(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API XsVector* XsDataPacket_rawGyroscopeDataConverted(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGyroscopeData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGyroscopeData(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawGyroscopeTemperatureData(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API XsVector* XsDataPacket_rawGyroscopeTemperatureDataConverted(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGyroscopeTemperatureData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGyroscopeTemperatureData(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API XsUShortVector* XsDataPacket_rawMagneticField(const XsDataPacket* thisPtr, XsUShortVector* returnVal);
XSTYPES_DLL_API XsVector* XsDataPacket_rawMagneticFieldConverted(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawMagneticField(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawMagneticField(XsDataPacket* thisPtr, const XsUShortVector* vec);
XSTYPES_DLL_API uint16_t XsDataPacket_rawTemperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsRawTemperature(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawTemperature(XsDataPacket* thisPtr, uint16_t temp);
XSTYPES_DLL_API XsScrData* XsDataPacket_rawData(const XsDataPacket* thisPtr, XsScrData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawData(XsDataPacket* thisPtr, const XsScrData* data);
XSTYPES_DLL_API XsVector* XsDataPacket_velocityIncrement(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsVelocityIncrement(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setVelocityIncrement(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedAcceleration(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedAcceleration(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsQuaternion* XsDataPacket_orientationIncrement(const XsDataPacket* thisPtr, XsQuaternion* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsOrientationIncrement(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setOrientationIncrement(XsDataPacket* thisPtr, const XsQuaternion* quat);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedGyroscopeData(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedGyroscopeData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedGyroscopeData(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsVector* XsDataPacket_calibratedMagneticField(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCalibratedMagneticField(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCalibratedMagneticField(XsDataPacket* thisPtr, const XsVector* vec);
XSTYPES_DLL_API XsVector* XsDataPacket_correctedMagneticField(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsCorrectedMagneticField(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setCorrectedMagneticField(XsDataPacket* thisPtr, const XsVector* vec);
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
XSTYPES_DLL_API XsGloveData* XsDataPacket_gloveData(const XsDataPacket* thisPtr, XsGloveData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsGloveData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setGloveData(XsDataPacket* thisPtr, const XsGloveData* data);
XSTYPES_DLL_API XsDeviceId* XsDataPacket_storedDeviceId(const XsDataPacket* thisPtr, XsDeviceId* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsStoredDeviceId(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setStoredDeviceId(XsDataPacket* thisPtr, const XsDeviceId* data);
XSTYPES_DLL_API uint16_t XsDataPacket_storedLocationId(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsStoredLocationId(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setStoredLocationId(XsDataPacket* thisPtr, uint16_t data);
XSTYPES_DLL_API uint32_t XsDataPacket_status(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsStatus(const XsDataPacket* thisPtr);
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
XSTYPES_DLL_API void XsDataPacket_setTemperature(XsDataPacket* thisPtr, double temperature);
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
XSTYPES_DLL_API double XsDataPacket_altitudeMsl(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsAltitudeMsl(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAltitudeMsl(XsDataPacket* thisPtr, double data);
XSTYPES_DLL_API XsVector* XsDataPacket_velocity(const XsDataPacket* thisPtr, XsVector* returnVal, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API int XsDataPacket_containsVelocity(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setVelocity(XsDataPacket* thisPtr, const XsVector* data, XsDataIdentifier coordinateSystem);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_velocityIdentifier(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsDataIdentifier XsDataPacket_coordinateSystemVelocity(const XsDataPacket* thisPtr);
XSTYPES_DLL_API XsTimeInfo* XsDataPacket_utcTime(const XsDataPacket* thisPtr, XsTimeInfo* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsUtcTime(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setUtcTime(XsDataPacket* thisPtr, const XsTimeInfo* data);
XSTYPES_DLL_API XsRange* XsDataPacket_frameRange(const XsDataPacket* thisPtr, XsRange* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsFrameRange(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setFrameRange(XsDataPacket* thisPtr, const XsRange* r);
XSTYPES_DLL_API int XsDataPacket_rssi(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsRssi(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRssi(XsDataPacket* thisPtr, int r);

XSTYPES_DLL_API XsRawGnssPvtData* XsDataPacket_rawGnssPvtData(const XsDataPacket* thisPtr, XsRawGnssPvtData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGnssPvtData(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGnssPvtData(XsDataPacket* thisPtr, const XsRawGnssPvtData* r);
XSTYPES_DLL_API uint8_t XsDataPacket_gnssAge(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_containsGnssAge(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setGnssAge(XsDataPacket* thisPtr, uint8_t age);
XSTYPES_DLL_API XsRawGnssSatInfo* XsDataPacket_rawGnssSatInfo(const XsDataPacket* thisPtr, XsRawGnssSatInfo* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawGnssSatInfo(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRawGnssSatInfo(XsDataPacket* thisPtr, const XsRawGnssSatInfo* r);

XSTYPES_DLL_API XsDataPacket* XsDataPacket_merge(XsDataPacket* thisPtr, const XsDataPacket* other, int overwrite);
XSTYPES_DLL_API void XsDataPacket_setTriggerIndication(XsDataPacket* thisPtr, XsDataIdentifier triggerId, const XsTriggerIndicationData * triggerIndicationData);
XSTYPES_DLL_API XsTriggerIndicationData* XsDataPacket_triggerIndication(const XsDataPacket* thisPtr, XsDataIdentifier triggerId, XsTriggerIndicationData* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsTriggerIndication(const XsDataPacket* thisPtr, XsDataIdentifier triggerId);
XSTYPES_DLL_API void XsDataPacket_toMessage(const XsDataPacket* thisPtr, XsMessage* msg);

XSTYPES_DLL_API void XsDataPacket_setAwindaSnapshot(XsDataPacket* thisPtr, XsSnapshot const * data, int retransmission);
XSTYPES_DLL_API XsSnapshot* XsDataPacket_awindaSnapshot(const XsDataPacket* thisPtr, XsSnapshot* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsAwindaSnapshot(const XsDataPacket* thisPtr);
XSTYPES_DLL_API int XsDataPacket_isAwindaSnapshotARetransmission(const XsDataPacket* thisPtr);

XSTYPES_DLL_API void XsDataPacket_setFullSnapshot(XsDataPacket* thisPtr, XsSnapshot const * data, int retransmission);
XSTYPES_DLL_API XsSnapshot* XsDataPacket_fullSnapshot(const XsDataPacket* thisPtr, XsSnapshot* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsFullSnapshot(const XsDataPacket* thisPtr);

XSTYPES_DLL_API void XsDataPacket_setGloveSnapshot(XsDataPacket* thisPtr, XsGloveSnapshot const * data, int retransmission);
XSTYPES_DLL_API XsGloveSnapshot* XsDataPacket_gloveSnapshot(const XsDataPacket* thisPtr, XsGloveSnapshot* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsGloveSnapshot(const XsDataPacket* thisPtr);

XSTYPES_DLL_API void XsDataPacket_setRawBlob(XsDataPacket* thisPtr, const XsByteArray * data);
XSTYPES_DLL_API XsByteArray* XsDataPacket_rawBlob(const XsDataPacket* thisPtr, XsByteArray* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRawBlob(const XsDataPacket* thisPtr);

XSTYPES_DLL_API XsVector* XsDataPacket_accelerationHR(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsAccelerationHR(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setAccelerationHR(XsDataPacket* thisPtr, const XsVector* vec);

XSTYPES_DLL_API XsVector* XsDataPacket_rateOfTurnHR(const XsDataPacket* thisPtr, XsVector* returnVal);
XSTYPES_DLL_API int XsDataPacket_containsRateOfTurnHR(const XsDataPacket* thisPtr);
XSTYPES_DLL_API void XsDataPacket_setRateOfTurnHR(XsDataPacket* thisPtr, const XsVector* vec);

XSTYPES_DLL_API int XsDataPacket_privateCount();

#ifdef __cplusplus
} // extern "C"
#ifndef XSENS_NO_STL
#include <map>	// only needed for simplifiedContents()
#endif
#endif

struct XsDataPacket {
#ifdef __cplusplus
	/*! \brief Default constructor, initializes empty data packet or from the supplied \a msg
		\param msg Either 0 to create an empty object or a pointer to a valid %XsMessage containing
		MTData2 data.
	*/
	inline explicit XsDataPacket(const XsMessage* msg = 0)
	{
		XsDataPacket_construct(this);
		if (msg)
			XsDataPacket_setMessage(this, msg);
	}

	/*! \brief Copy constructor
		\param pack The packet to copy from
	*/
	inline XsDataPacket(const XsDataPacket& pack)
	{
		XsDataPacket_copyConstruct(this, &pack);
	}

	//! \copydoc XsDataPacket_destruct
	inline ~XsDataPacket()
	{
		XsDataPacket_destruct(this);
	}

	/*! \brief Assignment operator
		\param other The packet to copy from
		\returns A reference to this %XsDataPacket
		\sa XsDataPacket_copy
	*/
	inline XsDataPacket& operator = (const XsDataPacket& other)
	{
		if (this != &other)
			XsDataPacket_copy(this, &other);
		return *this;
	}

	/*! \copydoc XsDataPacket_swap(XsDataPacket*,XsDataPacket*)*/
	inline void swap(XsDataPacket& other)
	{
		XsDataPacket_swap(this, &other);
	}

	/*! \copydoc XsDataPacket_clear(XsDataPacket*,XsDataIdentifier)*/
	inline void clear(XsDataIdentifier id = XDI_None)
	{
		XsDataPacket_clear(this, id);
	}

	/*! \copydoc XsDataPacket_empty(const XsDataPacket*)*/
	inline bool empty(void) const
	{
		return 0 != XsDataPacket_empty(this);
	}

	//! \brief Return the device ID associated with the data packet
	inline XsDeviceId deviceId() const
	{
		return m_deviceId;
	}

	/*! \copydoc XsDataPacket_itemCount(const XsDataPacket*)*/
	inline uint16_t itemCount() const
	{
		return static_cast<uint16_t>(static_cast<unsigned int>(XsDataPacket_itemCount(this)));
	}

	//! \copydoc XsDataPacket_setMessage(XsDataPacket*, const XsMessage*)
	inline void setMessage(const XsMessage& msg)
	{
		XsDataPacket_setMessage(this, &msg);
	}

	/*! \brief Returns a const reference to the message that contains the data packet
	*/
	inline XsMessage toMessage() const
	{
		XsMessage msg;
		XsDataPacket_toMessage(this, &msg);
		return msg;
	}

	/*! \brief Set the device ID associated with this data packet
		\param id The device ID to set
	*/
	inline void setDeviceId(const XsDeviceId id)
	{
		m_deviceId = id;
	}

	/*! \copydoc XsDataPacket_dataFormat(const XsDataPacket*, XsDataIdentifier) */
	inline XsDataIdentifier dataFormat(XsDataIdentifier id) const
	{
		return XsDataPacket_dataFormat(this, id);
	}

	/*! \brief \copybrief XsDataPacket_rawAcceleration(const XsDataPacket*, XsUShortVector*) */
	inline XsUShortVector rawAcceleration(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawAcceleration(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawAccelerationConverted(const XsDataPacket*, XsVector*) */
	inline XsVector rawAccelerationConverted(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rawAccelerationConverted(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawAcceleration(const XsDataPacket*) */
	inline bool containsRawAcceleration(void) const
	{
		return 0 != XsDataPacket_containsRawAcceleration(this);
	}

	/*! \copydoc XsDataPacket_setRawAcceleration(XsDataPacket*, const XsUShortVector*) */
	inline void setRawAcceleration(const XsUShortVector& vec)
	{
		XsDataPacket_setRawAcceleration(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeData(const XsDataPacket*, XsUShortVector*) */
	inline XsUShortVector rawGyroscopeData(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawGyroscopeData(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeDataConverted(const XsDataPacket*, XsVector*) */
	inline XsVector rawGyroscopeDataConverted(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rawGyroscopeDataConverted(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawGyroscopeData(const XsDataPacket*) */
	inline bool containsRawGyroscopeData(void) const
	{
		return 0 != XsDataPacket_containsRawGyroscopeData(this);
	}

	/*! \copydoc XsDataPacket_setRawGyroscopeData(XsDataPacket*, const XsUShortVector*) */
	inline void setRawGyroscopeData(const XsUShortVector& vec)
	{
		XsDataPacket_setRawGyroscopeData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeTemperatureData(const XsDataPacket*, XsUShortVector*) */
	inline XsUShortVector rawGyroscopeTemperatureData(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawGyroscopeTemperatureData(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawGyroscopeTemperatureDataConverted(const XsDataPacket*, XsVector*) */
	inline XsVector rawGyroscopeTemperatureDataConverted(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rawGyroscopeTemperatureDataConverted(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawGyroscopeTemperatureData(const XsDataPacket*) */
	inline bool containsRawGyroscopeTemperatureData(void) const
	{
		return 0 != XsDataPacket_containsRawGyroscopeTemperatureData(this);
	}

	/*! \copydoc XsDataPacket_setRawGyroscopeTemperatureData(XsDataPacket*, const XsUShortVector*) */
	inline void setRawGyroscopeTemperatureData(const XsUShortVector& vec)
	{
		XsDataPacket_setRawGyroscopeTemperatureData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawMagneticField(const XsDataPacket*, XsUShortVector*) */
	inline XsUShortVector rawMagneticField(void) const
	{
		XsUShortVector returnVal;
		return *XsDataPacket_rawMagneticField(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_rawMagneticFieldConverted(const XsDataPacket*, XsVector*) */
	inline XsVector rawMagneticFieldConverted(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rawMagneticFieldConverted(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawMagneticField(const XsDataPacket*) */
	inline bool containsRawMagneticField(void) const
	{
		return 0 != XsDataPacket_containsRawMagneticField(this);
	}

	/*! \copydoc XsDataPacket_setRawMagneticField(XsDataPacket*, const XsUShortVector*) */
	inline void setRawMagneticField(const XsUShortVector& vec)
	{
		XsDataPacket_setRawMagneticField(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rawTemperature(const XsDataPacket*) */
	inline uint16_t rawTemperature(void) const
	{
		return XsDataPacket_rawTemperature(this);
	}

	/*! \copydoc XsDataPacket_containsRawTemperature(const XsDataPacket*) */
	inline bool containsRawTemperature(void) const
	{
		return 0 != XsDataPacket_containsRawTemperature(this);
	}

	/*! \copydoc XsDataPacket_setRawTemperature(XsDataPacket*, uint16_t) */
	inline void setRawTemperature(uint16_t temp)
	{
		XsDataPacket_setRawTemperature(this, temp);
	}

	/*! \brief \copybrief XsDataPacket_rawData(const XsDataPacket*, XsScrData*)
		\return The raw data component of a data item.
	*/
	inline XsScrData rawData(void) const
	{
		XsScrData returnVal;
		return *XsDataPacket_rawData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRawData(const XsDataPacket*) */
	inline bool containsRawData(void) const
	{
		return 0 != XsDataPacket_containsRawData(this);
	}

	/*! \copydoc XsDataPacket_setRawData(XsDataPacket*, const XsScrData*) */
	inline void setRawData(const XsScrData& data)
	{
		XsDataPacket_setRawData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_velocityIncrement(const XsDataPacket*, XsVector*) */
	inline XsVector velocityIncrement(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_velocityIncrement(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsVelocityIncrement(const XsDataPacket*) */
	inline bool containsVelocityIncrement(void) const
	{
		return 0 != XsDataPacket_containsVelocityIncrement(this);
	}

	/*! \copydoc XsDataPacket_setVelocityIncrement(XsDataPacket*, const XsVector*) */
	inline void setVelocityIncrement(const XsVector& vec)
	{
		XsDataPacket_setVelocityIncrement(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedAcceleration(const XsDataPacket*, XsVector*) */
	inline XsVector calibratedAcceleration(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedAcceleration(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedAcceleration(const XsDataPacket*) */
	inline bool containsCalibratedAcceleration(void) const
	{
		return 0 != XsDataPacket_containsCalibratedAcceleration(this);
	}

	/*! \copydoc XsDataPacket_setCalibratedAcceleration(XsDataPacket*, const XsVector*) */
	inline void setCalibratedAcceleration(const XsVector& vec)
	{
		XsDataPacket_setCalibratedAcceleration(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_orientationIncrement(const XsDataPacket*, XsQuaternion*) */
	inline XsQuaternion orientationIncrement(void) const
	{
		XsQuaternion returnVal;
		return *XsDataPacket_orientationIncrement(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsOrientationIncrement(const XsDataPacket*) */
	inline bool containsOrientationIncrement(void) const
	{
		return 0 != XsDataPacket_containsOrientationIncrement(this);
	}

	/*! \copydoc XsDataPacket_setOrientationIncrement(XsDataPacket*, const XsQuaternion*) */
	inline void setOrientationIncrement(const XsQuaternion& quat)
	{
		XsDataPacket_setOrientationIncrement(this, &quat);
	}

	/*! \brief \copybrief XsDataPacket_calibratedGyroscopeData(const XsDataPacket*, XsVector*) */
	inline XsVector calibratedGyroscopeData(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedGyroscopeData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedGyroscopeData(const XsDataPacket*) */
	inline bool containsCalibratedGyroscopeData(void) const
	{
		return 0 != XsDataPacket_containsCalibratedGyroscopeData(this);
	}

	/*! \copydoc XsDataPacket_setCalibratedGyroscopeData(XsDataPacket*, const XsVector*) */
	inline void setCalibratedGyroscopeData(const XsVector& vec)
	{
		XsDataPacket_setCalibratedGyroscopeData(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedMagneticField(const XsDataPacket*, XsVector*) */
	inline XsVector calibratedMagneticField(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_calibratedMagneticField(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedMagneticField(const XsDataPacket*) */
	inline bool containsCalibratedMagneticField(void) const
	{
		return 0 != XsDataPacket_containsCalibratedMagneticField(this);
	}

	/*! \copydoc XsDataPacket_setCalibratedMagneticField(XsDataPacket*, const XsVector*) */
	inline void setCalibratedMagneticField(const XsVector& vec)
	{
		XsDataPacket_setCalibratedMagneticField(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_correctedMagneticField(const XsDataPacket*, XsVector*) */
	XSNOEXPORT inline XsVector correctedMagneticField(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_correctedMagneticField(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCorrectedMagneticField(const XsDataPacket*) */
	XSNOEXPORT inline bool containsCorrectedMagneticField(void) const
	{
		return 0 != XsDataPacket_containsCorrectedMagneticField(this);
	}

	/*! \copydoc XsDataPacket_setCorrectedMagneticField(XsDataPacket*, const XsVector*) */
	XSNOEXPORT inline void setCorrectedMagneticField(const XsVector& vec)
	{
		XsDataPacket_setCorrectedMagneticField(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_calibratedData(const XsDataPacket*, XsCalibratedData*) */
	inline XsCalibratedData calibratedData(void) const
	{
		XsCalibratedData returnVal;
		return *XsDataPacket_calibratedData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsCalibratedData(const XsDataPacket*) */
	inline bool containsCalibratedData(void) const
	{
		return 0 != XsDataPacket_containsCalibratedData(this);
	}

	/*! \copydoc XsDataPacket_setCalibratedData(XsDataPacket*, const XsCalibratedData*) */
	inline void setCalibratedData(const XsCalibratedData& data)
	{
		XsDataPacket_setCalibratedData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_orientationQuaternion(const XsDataPacket*, XsQuaternion*, XsDataIdentifier)
		\param coordinateSystem The coordinate system of the requested orientation. If this does not match
			the stored coordinate system, it will be transformed to the requested orientation.
		\returns The requested data
	*/
	inline XsQuaternion orientationQuaternion(XsDataIdentifier coordinateSystem) const
	{
		XsQuaternion returnVal;
		return *XsDataPacket_orientationQuaternion(this, &returnVal, coordinateSystem);
	}

	/*! \brief Returns the orientation as a quaternion with the current coordinate system
	*/
	inline XsQuaternion orientationQuaternion() const
	{
		XsQuaternion returnVal;
		return *XsDataPacket_orientationQuaternion(this, &returnVal, coordinateSystemOrientation());
	}

	/*! \copydoc XsDataPacket_setOrientationQuaternion(XsDataPacket*, const XsQuaternion*, XsDataIdentifier) */
	inline void setOrientationQuaternion(const XsQuaternion& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationQuaternion(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_orientationEuler(const XsDataPacket*, XsEuler*, XsDataIdentifier)
		\param coordinateSystem The coordinate system of the requested orientation. If this does not match
				the stored coordinate system, it will be transformed to the requested orientation.
		\returns The requested data
	*/
	inline XsEuler orientationEuler(XsDataIdentifier coordinateSystem) const
	{
		XsEuler returnVal;
		return *XsDataPacket_orientationEuler(this, &returnVal, coordinateSystem);
	}

	/*! \brief Returns the orientation as an XsEuler with the current coordinate system*/
	inline XsEuler orientationEuler() const
	{
		XsEuler returnVal;
		return *XsDataPacket_orientationEuler(this, &returnVal, coordinateSystemOrientation());
	}

	/*! \copydoc XsDataPacket_setOrientationEuler(XsDataPacket*, const XsEuler*, XsDataIdentifier) */
	inline void setOrientationEuler(const XsEuler& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationEuler(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_orientationMatrix(const XsDataPacket*, XsMatrix*, XsDataIdentifier)
		\param coordinateSystem The coordinate system of the requested orientation. If this does not match
			the stored coordinate system, it will be transformed to the requested orientation.
		\returns The requested data
	*/
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

	/*! \copydoc XsDataPacket_setOrientationMatrix(XsDataPacket*, const XsMatrix*, XsDataIdentifier) */
	inline void setOrientationMatrix(const XsMatrix& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setOrientationMatrix(this, &data, coordinateSystem);
	}

	/*! \copydoc XsDataPacket_containsOrientation(const XsDataPacket*) */
	inline bool containsOrientation(void) const
	{
		return 0 != XsDataPacket_containsOrientation(this);
	}

	/*! \copydoc XsDataPacket_orientationIdentifier(const XsDataPacket*) */
	inline XsDataIdentifier orientationIdentifier() const
	{
		return XsDataPacket_orientationIdentifier(this);
	}

	/*! \copydoc XsDataPacket_coordinateSystemOrientation(const XsDataPacket*) */
	inline XsDataIdentifier coordinateSystemOrientation() const
	{
		return XsDataPacket_coordinateSystemOrientation(this);
	}

	/*! \brief \copybrief XsDataPacket_sdiData(const XsDataPacket*, XsSdiData*) */
	inline XsSdiData sdiData(void) const
	{
		XsSdiData returnVal;
		return *XsDataPacket_sdiData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsSdiData(const XsDataPacket*) */
	inline bool containsSdiData(void) const
	{
		return 0 != XsDataPacket_containsSdiData(this);
	}

	/*! \copydoc XsDataPacket_setSdiData(XsDataPacket*, const XsSdiData*) */
	inline void setSdiData(const XsSdiData& data)
	{
		XsDataPacket_setSdiData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_sdiData(const XsDataPacket*, XsSdiData*) */
	inline XsGloveData gloveData(void) const
	{
		XsGloveData returnVal;
		return *XsDataPacket_gloveData(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsSdiData(const XsDataPacket*) */
	inline bool containsGloveData(void) const
	{
		return 0 != XsDataPacket_containsGloveData(this);
	}

	/*! \copydoc XsDataPacket_setGloveData(XsDataPacket*, const XsGloveData*) */
	XSNOEXPORT inline void setGloveData(const XsGloveData& data)
	{
		XsDataPacket_setGloveData(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_storedDeviceId(const XsDataPacket*, XsDeviceId*)
	  \returns the device ID stored in this packet
	*/
	inline XsDeviceId storedDeviceId(void) const
	{
		XsDeviceId returnVal;
		return *XsDataPacket_storedDeviceId(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsStoredDeviceId(const XsDataPacket*) */
	inline bool containsStoredDeviceId(void) const
	{
		return 0 != XsDataPacket_containsStoredDeviceId(this);
	}

	/*! \copydoc XsDataPacket_setStoredDeviceId(XsDataPacket*, const XsDeviceId*) */
	inline void setStoredDeviceId(const XsDeviceId& data)
	{
		XsDataPacket_setStoredDeviceId(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_storedLocationId(const XsDataPacket*)
	\returns the location ID stored in this packet
	*/
	inline uint16_t storedLocationId(void) const
	{
		return XsDataPacket_storedLocationId(this);
	}

	/*! \copydoc XsDataPacket_containsStoredLocationId(const XsDataPacket*) */
	inline bool containsStoredLocationId(void) const
	{
		return 0 != XsDataPacket_containsStoredLocationId(this);
	}

	/*! \copydoc XsDataPacket_setStoredLocationId(XsDataPacket*, uint16_t) */
	inline void setStoredLocationId(uint16_t data)
	{
		XsDataPacket_setStoredLocationId(this, data);
	}


	/*! \brief \copybrief XsDataPacket_status(const XsDataPacket*) */
	inline uint32_t status(void) const
	{
		return XsDataPacket_status(this);
	}

	/*! \copydoc XsDataPacket_containsStatus(const XsDataPacket*) */
	inline bool containsStatus(void) const
	{
		return 0 != XsDataPacket_containsStatus(this);
	}

	/*! \copydoc XsDataPacket_containsDetailedStatus(const XsDataPacket*) */
	inline bool containsDetailedStatus(void) const
	{
		return 0 != XsDataPacket_containsDetailedStatus(this);
	}

	/*! \copydoc XsDataPacket_setStatus(XsDataPacket*, uint32_t) */
	inline void setStatus(const uint32_t data)
	{
		XsDataPacket_setStatus(this, data);
	}

	/*! \copydoc XsDataPacket_setStatusByte(XsDataPacket*, uint8_t) */
	inline void setStatusByte(const uint8_t data)
	{
		XsDataPacket_setStatusByte(this, data);
	}

	/*! \brief \copybrief XsDataPacket_packetCounter8(const XsDataPacket*) */
	inline uint8_t packetCounter8(void) const
	{
		return XsDataPacket_packetCounter8(this);
	}

	/*! \copydoc XsDataPacket_containsPacketCounter8(const XsDataPacket*) */
	inline bool containsPacketCounter8(void) const
	{
		return 0 != XsDataPacket_containsPacketCounter8(this);
	}

	/*! \copydoc XsDataPacket_setPacketCounter8(XsDataPacket*, uint8_t) */
	inline void setPacketCounter8(uint8_t counter)
	{
		XsDataPacket_setPacketCounter8(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_packetCounter(const XsDataPacket* thisPtr) */
	inline uint16_t packetCounter(void) const
	{
		return XsDataPacket_packetCounter(this);
	}

	/*! \copydoc XsDataPacket_containsPacketCounter(const XsDataPacket*) */
	inline bool containsPacketCounter(void) const
	{
		return 0 != XsDataPacket_containsPacketCounter(this);
	}

	/*! \copydoc XsDataPacket_setPacketCounter(XsDataPacket*, uint16_t) */
	inline void setPacketCounter(uint16_t counter)
	{
		XsDataPacket_setPacketCounter(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTimeFine(const XsDataPacket*) */
	inline uint32_t sampleTimeFine(void) const
	{
		return XsDataPacket_sampleTimeFine(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTimeFine(const XsDataPacket*) */
	inline bool containsSampleTimeFine(void) const
	{
		return 0 != XsDataPacket_containsSampleTimeFine(this);
	}

	/*! \copydoc XsDataPacket_setSampleTimeFine(XsDataPacket*, uint32_t) */
	inline void setSampleTimeFine(uint32_t counter)
	{
		XsDataPacket_setSampleTimeFine(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTimeCoarse(const XsDataPacket*) */
	inline uint32_t sampleTimeCoarse(void) const
	{
		return XsDataPacket_sampleTimeCoarse(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTimeCoarse(const XsDataPacket*) */
	inline bool containsSampleTimeCoarse(void) const
	{
		return 0 != XsDataPacket_containsSampleTimeCoarse(this);
	}

	/*! \copydoc XsDataPacket_setSampleTimeCoarse(XsDataPacket*, uint32_t) */
	inline void setSampleTimeCoarse(uint32_t counter)
	{
		XsDataPacket_setSampleTimeCoarse(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_sampleTime64(const XsDataPacket*) */
	inline uint64_t sampleTime64(void) const
	{
		return XsDataPacket_sampleTime64(this);
	}

	/*! \copydoc XsDataPacket_containsSampleTime64(const XsDataPacket*) */
	inline bool containsSampleTime64(void) const
	{
		return 0 != XsDataPacket_containsSampleTime64(this);
	}

	/*! \copydoc XsDataPacket_setSampleTime64(XsDataPacket*, uint64_t) */
	inline void setSampleTime64(uint64_t counter)
	{
		XsDataPacket_setSampleTime64(this, counter);
	}

	/*! \brief \copybrief XsDataPacket_freeAcceleration(const XsDataPacket*, XsVector*) */
	inline XsVector freeAcceleration(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_freeAcceleration(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsFreeAcceleration(const XsDataPacket*) */
	inline bool containsFreeAcceleration(void) const
	{
		return 0 != XsDataPacket_containsFreeAcceleration(this);
	}

	/*! \copydoc XsDataPacket_setFreeAcceleration(XsDataPacket*, const XsVector*) */
	inline void setFreeAcceleration(const XsVector& g)
	{
		XsDataPacket_setFreeAcceleration(this, &g);
	}

	/*! \brief \copybrief XsDataPacket_temperature(const XsDataPacket*) */
	inline double temperature(void) const
	{
		return XsDataPacket_temperature(this);
	}

	/*! \copydoc XsDataPacket_containsTemperature(const XsDataPacket*) */
	inline bool containsTemperature(void) const
	{
		return 0 != XsDataPacket_containsTemperature(this);
	}

	/*! \copydoc XsDataPacket_setTemperature(XsDataPacket*, double) */
	inline void setTemperature(double temperature)
	{
		XsDataPacket_setTemperature(this, temperature);
	}

	/*! \brief \copybrief XsDataPacket_pressure(const XsDataPacket*, XsPressure*) */
	inline XsPressure pressure(void) const
	{
		XsPressure returnVal;
		return *XsDataPacket_pressure(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsPressure(const XsDataPacket*) */
	inline bool containsPressure(void) const
	{
		return 0 != XsDataPacket_containsPressure(this);
	}

	/*! \copydoc XsDataPacket_containsPressureAge(const XsDataPacket*) */
	inline bool containsPressureAge(void) const
	{
		return 0 != XsDataPacket_containsPressureAge(this);
	}

	/*! \copydoc XsDataPacket_setPressure(XsDataPacket*, const XsPressure*) */
	inline void setPressure(const XsPressure& data)
	{
		XsDataPacket_setPressure(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_analogIn1Data(const XsDataPacket*, XsAnalogInData*) */
	inline XsAnalogInData analogIn1Data(void) const
	{
		XsAnalogInData returnVal;
		return *XsDataPacket_analogIn1Data(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsAnalogIn1Data(const XsDataPacket*) */
	inline bool containsAnalogIn1Data(void) const
	{
		return 0 != XsDataPacket_containsAnalogIn1Data(this);
	}

	/*! \copydoc XsDataPacket_setAnalogIn1Data(XsDataPacket*, const XsAnalogInData*) */
	inline void setAnalogIn1Data(const XsAnalogInData& data)
	{
		XsDataPacket_setAnalogIn1Data(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_analogIn2Data(const XsDataPacket*, XsAnalogInData*) */
	inline XsAnalogInData analogIn2Data(void) const
	{
		XsAnalogInData returnVal;
		return *XsDataPacket_analogIn2Data(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsAnalogIn2Data(const XsDataPacket*) */
	inline bool containsAnalogIn2Data(void) const
	{
		return 0 != XsDataPacket_containsAnalogIn2Data(this);
	}

	/*! \copydoc XsDataPacket_setAnalogIn2Data(XsDataPacket*, const XsAnalogInData*) */
	inline void setAnalogIn2Data(const XsAnalogInData& data)
	{
		XsDataPacket_setAnalogIn2Data(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_positionLLA(const XsDataPacket*, XsVector*) */
	inline XsVector positionLLA(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_positionLLA(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsPositionLLA(const XsDataPacket*) */
	inline bool containsPositionLLA(void) const
	{
		return 0 != XsDataPacket_containsPositionLLA(this);
	}

	/*! \copydoc XsDataPacket_setPositionLLA(XsDataPacket*, const XsVector*) */
	inline void setPositionLLA(const XsVector& data)
	{
		XsDataPacket_setPositionLLA(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_latitudeLongitude(const XsDataPacket*, XsVector*) */
	inline XsVector latitudeLongitude(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_latitudeLongitude(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsLatitudeLongitude(const XsDataPacket*) */
	inline bool containsLatitudeLongitude(void) const
	{
		return 0 != XsDataPacket_containsLatitudeLongitude(this);
	}

	/*! \copydoc XsDataPacket_setLatitudeLongitude(XsDataPacket*, const XsVector*) */
	inline void setLatitudeLongitude(const XsVector& data)
	{
		XsDataPacket_setLatitudeLongitude(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_altitude(const XsDataPacket*) */
	inline double altitude(void) const
	{
		return XsDataPacket_altitude(this);
	}

	/*! \copydoc XsDataPacket_containsAltitude(const XsDataPacket*) */
	inline bool containsAltitude(void) const
	{
		return 0 != XsDataPacket_containsAltitude(this);
	}

	/*! \copydoc XsDataPacket_setAltitude(XsDataPacket*, double) */
	inline void setAltitude(double data)
	{
		XsDataPacket_setAltitude(this, data);
	}

	/*! \brief \copybrief XsDataPacket_altitudeMsl(const XsDataPacket*) */
	inline double altitudeMsl(void) const
	{
		return XsDataPacket_altitudeMsl(this);
	}

	/*! \copydoc XsDataPacket_containsAltitudeMsl(const XsDataPacket*) */
	inline bool containsAltitudeMsl(void) const
	{
		return 0 != XsDataPacket_containsAltitudeMsl(this);
	}

	/*! \copydoc XsDataPacket_setAltitudeMsl(XsDataPacket*, double) */
	inline void setAltitudeMsl(double data)
	{
		XsDataPacket_setAltitudeMsl(this, data);
	}

	/*! \brief \copybrief XsDataPacket_velocity(const XsDataPacket*, XsVector*, XsDataIdentifier)
		\param coordinateSystem The coordinate system of the requested velocity. If this does not match
			the stored coordinate system, it will be transformed to the requested velocity.
		\returns A XsVector containing the x, y and z axis values in that order
	*/
	inline XsVector velocity(XsDataIdentifier coordinateSystem) const
	{
		XsVector returnVal;
		return *XsDataPacket_velocity(this, &returnVal, coordinateSystem);
	}

	/*! \brief Returns the velocity with the current coordinate system*/
	inline XsVector velocity(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_velocity(this, &returnVal, coordinateSystemVelocity());
	}

	/*! \copydoc XsDataPacket_containsVelocity(const XsDataPacket*) */
	inline bool containsVelocity(void) const
	{
		return 0 != XsDataPacket_containsVelocity(this);
	}

	/*! \copydoc XsDataPacket_setVelocity(XsDataPacket*, const XsVector*, XsDataIdentifier) */
	inline void setVelocity(const XsVector& data, XsDataIdentifier coordinateSystem)
	{
		XsDataPacket_setVelocity(this, &data, coordinateSystem);
	}

	/*! \brief \copybrief XsDataPacket_velocityIdentifier(const XsDataPacket*) */
	inline XsDataIdentifier velocityIdentifier() const
	{
		return XsDataPacket_velocityIdentifier(this);
	}

	/*! \copydoc XsDataPacket_coordinateSystemVelocity(const XsDataPacket*) */
	inline XsDataIdentifier coordinateSystemVelocity() const
	{
		return XsDataPacket_coordinateSystemVelocity(this);
	}

	/*! \brief \copybrief XsDataPacket_utcTime(const XsDataPacket*, XsTimeInfo*)
		\returns An XsTimeInfo containing the utc time value
	*/
	inline XsTimeInfo utcTime(void) const
	{
		XsTimeInfo returnVal;
		return *XsDataPacket_utcTime(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsUtcTime(const XsDataPacket*) */
	inline bool containsUtcTime(void) const
	{
		return 0 != XsDataPacket_containsUtcTime(this);
	}

	/*! \copydoc XsDataPacket_setUtcTime(XsDataPacket*, const XsTimeInfo*) */
	inline void setUtcTime(const XsTimeInfo& data)
	{
		XsDataPacket_setUtcTime(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_frameRange(const XsDataPacket*, XsRange*) */
	inline XsRange frameRange() const
	{
		XsRange returnVal;
		return *XsDataPacket_frameRange(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsFrameRange(const XsDataPacket*) */
	inline bool containsFrameRange() const
	{
		return 0 != XsDataPacket_containsFrameRange(this);
	}

	/*! \copydoc XsDataPacket_setFrameRange(XsDataPacket*, const XsRange*) */
	inline void setFrameRange(const XsRange& r)
	{
		XsDataPacket_setFrameRange(this, &r);
	}

	/*! \brief \copybrief XsDataPacket_rssi(const XsDataPacket*) */
	inline int rssi() const
	{
		return XsDataPacket_rssi(this);
	}

	/*! \copydoc XsDataPacket_containsRssi(const XsDataPacket*) */
	inline bool containsRssi() const
	{
		return 0 != XsDataPacket_containsRssi(this);
	}

	/*! \copydoc XsDataPacket_setRssi(XsDataPacket* thisPtr, int) */
	inline void setRssi(int r)
	{
		XsDataPacket_setRssi(this, r);
	}

	/*! \brief \copybrief XsDataPacket_rawGnssPvtData(const XsDataPacket*, XsRawGnssPvtData*)
		\return a struct with RawGnssPvtData
	*/
	inline XsRawGnssPvtData rawGnssPvtData(void) const
	{
		XsRawGnssPvtData returnVal;
		return *XsDataPacket_rawGnssPvtData(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGnssPvtData(const XsDataPacket*) */
	inline bool containsRawGnssPvtData(void) const
	{
		return 0 != XsDataPacket_containsRawGnssPvtData(this);
	}

	/*! \brief \copybrief XsDataPacket_setRawGnssPvtData */
	XSNOEXPORT inline void setRawGnssPvtData(XsRawGnssPvtData const& raw)
	{
		XsDataPacket_setRawGnssPvtData(this, &raw);
	}

	/*! \brief \copybrief XsDataPacket_gnssAge(const XsDataPacket*) */
	inline uint8_t gnssAge(void) const
	{
		return XsDataPacket_gnssAge(this);
	}

	/*! \brief \copybrief XsDataPacket_containsGnssAge(const XsDataPacket*) */
	inline bool containsGnssAge(void) const
	{
		return 0 != XsDataPacket_containsGnssAge(this);
	}

	/*! \brief \copybrief XsDataPacket_setGnssAge(XsDataPacket*, uint8_t) */
	XSNOEXPORT inline void setGnssAge(uint8_t age)
	{
		XsDataPacket_setGnssAge(this, age);
	}

	/*! \brief \copybrief XsDataPacket_rawGnssSatInfo(const XsDataPacket*, XsRawGnssSatInfo*)
		\return a struct with RawGnssSatInfo
	*/
	inline XsRawGnssSatInfo rawGnssSatInfo(void) const
	{
		XsRawGnssSatInfo returnVal;
		return *XsDataPacket_rawGnssSatInfo(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawGnssSatInfo(const XsDataPacket*) */
	inline bool containsRawGnssSatInfo(void) const
	{
		return 0 != XsDataPacket_containsRawGnssSatInfo(this);
	}

	/*! \copydoc XsDataPacket_setRawGnssSatInfo(XsDataPacket*, const XsRawGnssSatInfo*) */
	XSNOEXPORT inline void setRawGnssSatInfo(XsRawGnssSatInfo const& data)
	{
		XsDataPacket_setRawGnssSatInfo(this, &data);
	}
	/*! \brief \copybrief XsDataPacket_fullSnapshot(const XsDataPacket*, XsSnapshot*) */
	inline XsSnapshot fullSnapshot(void) const
	{
		XsSnapshot returnVal;
		return *XsDataPacket_fullSnapshot(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsFullSnapshot(const XsDataPacket*) */
	inline bool containsFullSnapshot(void) const
	{
		return 0 != XsDataPacket_containsFullSnapshot(this);
	}

	/*! \copydoc XsDataPacket_setFullSnapshot(XsDataPacket*, XsSnapshot const *, int) */
	XSNOEXPORT inline void setFullSnapshot(XsSnapshot const& data, bool retransmission)
	{
		XsDataPacket_setFullSnapshot(this, &data, retransmission?1:0);
	}
	/*! \brief \copybrief XsDataPacket_awindaSnapshot */
	XSNOEXPORT inline XsSnapshot awindaSnapshot(void) const
	{
		XsSnapshot returnVal;
		return *XsDataPacket_awindaSnapshot(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsAwindaSnapshot(const XsDataPacket*) */
	XSNOEXPORT inline bool containsAwindaSnapshot(void) const
	{
		return 0 != XsDataPacket_containsAwindaSnapshot(this);
	}

	/*! \brief \copybrief XsDataPacket_setAwindaSnapshot */
	XSNOEXPORT inline void setAwindaSnapshot(XsSnapshot const& raw, bool retransmission)
	{
		XsDataPacket_setAwindaSnapshot(this, &raw, retransmission?1:0);
	}

	/*! \brief \copybrief XsDataPacket_containsAwindaSnapshot(const XsDataPacket*) */
	inline bool isAwindaSnapshotARetransmission(void) const
	{
		return 0 != XsDataPacket_isAwindaSnapshotARetransmission(this);
	}

	/*! \brief \copybrief XsDataPacket_gloveSnapshot(const XsDataPacket*, XsGloveSnapshot*) */
	XSNOEXPORT inline XsGloveSnapshot gloveSnapshot(void) const
	{
		XsGloveSnapshot returnVal;
		return *XsDataPacket_gloveSnapshot(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsGloveSnapshot(const XsDataPacket*) */
	inline bool containsGloveSnapshot(void) const
	{
		return 0 != XsDataPacket_containsGloveSnapshot(this);
	}

	/*! \copydoc XsDataPacket_setGloveSnapshot(XsDataPacket*, XsGloveSnapshot const *, int) */
	XSNOEXPORT inline void setGloveSnapshot(XsGloveSnapshot const& data, bool retransmission)
	{
		XsDataPacket_setGloveSnapshot(this, &data, retransmission ? 1 : 0);
	}

	/*! \copydoc XsDataPacket_merge(XsDataPacket*, const XsDataPacket*, int) */
	inline XsDataPacket& merge(const XsDataPacket& other, bool overwrite = true)
	{
		return *XsDataPacket_merge(this, &other, overwrite?1:0);
	}

	/*! \brief Set the time of arrival of the data packet
		\param t The time of arrival
	*/
	inline void setTimeOfArrival(const XsTimeStamp& t)
	{
		m_toa = t;
	}

	/*! \brief Return the time of arrival of the data packet. Only valid for live streams. The behaviour for file streams is undefined and may change in the future. */
	inline XsTimeStamp timeOfArrival() const
	{
		return m_toa;
	}

	/*! \brief Set the estimated time of sampling of the data packet
		\param t The estimated time of sampling
	*/
	inline void setEstimatedTimeOfSampling(const XsTimeStamp& t)
	{
		m_etos = t;
	}

	/*! \brief Return the estimated time of sampling of the data packet. Only valid for live streams. The behaviour for file streams is undefined and may change in the future. */
	inline XsTimeStamp estimatedTimeOfSampling() const
	{
		return m_etos;
	}

	/*! \brief Set the packet ID of the data packet
		\param t The packet ID to set
	*/
	inline void setPacketId(int64_t t)
	{
		m_packetId = t;
	}

	/*! \brief Return the ID of the packet.
		\details This ID is based on, depending on availability: (1) packet counter (2) sample time (3) arrival order
		\returns The ID of the packet.
	*/
	inline int64_t packetId() const
	{
		return m_packetId;
	}

	/*! \copydoc XsDataPacket_setTriggerIndication(XsDataPacket*, XsDataIdentifier, const XsTriggerIndicationData *) */
	void setTriggerIndication(XsDataIdentifier triggerId, const XsTriggerIndicationData &triggerIndicationData)
	{
		XsDataPacket_setTriggerIndication(this, triggerId, &triggerIndicationData);
	}

	/*! \copydoc XsDataPacket_containsTriggerIndication(const XsDataPacket*, XsDataIdentifier) */
	inline bool containsTriggerIndication(XsDataIdentifier triggerId) const
	{
		return 0 != XsDataPacket_containsTriggerIndication(this, triggerId);
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

	/*! \brief \copybrief XsDataPacket_rawBlob(const XsDataPacket*, XsByteArray*) */
	inline XsByteArray rawBlob(void) const
	{
		XsByteArray returnVal;
		return *XsDataPacket_rawBlob(this, &returnVal);
	}

	/*! \brief \copybrief XsDataPacket_containsRawBlob(const XsDataPacket*) */
	inline bool containsRawBlob(void) const
	{
		return 0 != XsDataPacket_containsRawBlob(this);
	}

	/*! \copydoc XsDataPacket_setRawBlob(XsDataPacket*, const XsByteArray *) */
	inline void setRawBlob(const XsByteArray &data)
	{
		XsDataPacket_setRawBlob(this, &data);
	}

	/*! \brief \copybrief XsDataPacket_accelerationHR(const XsDataPacket*, XsVector*) */
	inline XsVector accelerationHR(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_accelerationHR(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsAccelerationHR(const XsDataPacket*) */
	inline bool containsAccelerationHR(void) const
	{
		return 0 != XsDataPacket_containsAccelerationHR(this);
	}

	/*! \copydoc XsDataPacket_setAccelerationHR(XsDataPacket*, const XsVector*) */
	inline void setAccelerationHR(const XsVector& vec)
	{
		XsDataPacket_setAccelerationHR(this, &vec);
	}

	/*! \brief \copybrief XsDataPacket_rateOfTurnHR(const XsDataPacket*, XsVector*) */
	inline XsVector rateOfTurnHR(void) const
	{
		XsVector returnVal;
		return *XsDataPacket_rateOfTurnHR(this, &returnVal);
	}

	/*! \copydoc XsDataPacket_containsRateOfTurnHR(const XsDataPacket*) */
	inline bool containsRateOfTurnHR(void) const
	{
		return 0 != XsDataPacket_containsRateOfTurnHR(this);
	}

	/*! \copydoc XsDataPacket_setRateOfTurnHR(XsDataPacket*, const XsVector*) */
	inline void setRateOfTurnHR(const XsVector& vec)
	{
		XsDataPacket_setRateOfTurnHR(this, &vec);
	}

#ifndef XSENS_NO_STL
	/*! \brief Return the contents of the XsDataPacket in a simplified form
		\details This function is meant for debugging purposes only and should not be used in production code.
		\return A STL map with the contents of the packet identified by XsDataIdentifier
	*/
	inline std::map<XsDataIdentifier, void*> simplifiedContents() const
	{
		return *((std::map<XsDataIdentifier, void*>*)(void*)d);
	}
#endif

//protected:
	/*! \privatesection */
#endif // __cplusplus
	struct XSNOEXPORT DataPacketPrivate* d;

	XsDeviceId			m_deviceId;					//!< The device Id to which the message belongs
	XsTimeStamp			m_toa;						//!< Time Of Arrival (live packets only)
	int64_t				m_packetId;					//!< 64 bit packet id, based on, depending on availability: (1) packet counter (2) sample time (3) arrival order
	XsTimeStamp			m_etos;						//!< Estimated Time of Sampling (live packets only)
};

#endif
