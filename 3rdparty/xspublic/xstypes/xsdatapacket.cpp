
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

#include "xsdatapacket.h"
#include "datapacket_p.h"
#include "xsmatrix3x3.h"
#include "xsvector.h"
#include "xsrssi.h"
#include "xsmath.h"
#include <string.h>

#define MAP	(*thisPtr->d)

static const uint64_t coarseFactor = 10000ULL;

using namespace XsDataPacket_Private;

static void detach(XsDataPacket* thisPtr)
{
	if (thisPtr->d->m_refCount == 1)
		return;
	DataPacketPrivate* old = thisPtr->d;
	thisPtr->d = new DataPacketPrivate(*old);
	if (--old->m_refCount == 0)	// this can happen in some concurrent situations
		delete old;
}

Variant* createVariant(XsDataIdentifier id)
{
	// It may be faster to create a static map with construct functions instead of this switch, but this is a much simpler implementation
	switch (id & XDI_FullTypeMask)
	{
	//XDI_TemperatureGroup		= 0x0800,
	case XDI_Temperature:
		return new SimpleVariant<double>(id);
	//case XDI_TimestampGroup		:// 0x1000,
	case XDI_UtcTime				:// 0x1010,
		return new XsTimeInfoVariant(id);
	case XDI_PacketCounter			:// 0x1020,
		return new SimpleVariant<uint16_t>(id);
	case XDI_Itow					:// 0x1030,
		return new SimpleVariant<uint32_t>(id);
	case XDI_GnssAge				:// 0x1040,
		return new SimpleVariant<uint8_t>(id);
	case XDI_PressureAge			:// 0x1050,
		return new SimpleVariant<uint8_t>(id);
	case XDI_SampleTimeFine			:// 0x1060,
	case XDI_SampleTimeCoarse		:// 0x1070,
		return new SimpleVariant<uint32_t>(id);
	case XDI_FrameRange				:// 0x1080,	// add for MTw (if needed)
		return new XsRangeVariant(id);
	case XDI_PacketCounter8			:// 0x1090,
		return new SimpleVariant<uint8_t>(id);
	case XDI_SampleTime64			:// 0x10A0,
		return new SimpleVariant<uint64_t>(id);

	//case XDI_OrientationGroup		:// 0x2000,
	case XDI_Quaternion				:// 0x2010,
		return new XsQuaternionVariant(id);
	case XDI_RotationMatrix			:// 0x2020,
		return new XsMatrixVariant(id);
	case XDI_EulerAngles			:// 0x2030,
		return new XsEulerVariant(id);

	//case XDI_PressureGroup		:// 0x3000,
	case XDI_BaroPressure			:// 0x3010,
		return new SimpleVariant<uint32_t>(id);

	//case XDI_AccelerationGroup	:// 0x4000,
	case XDI_DeltaV					:// 0x4010,
	case XDI_Acceleration			:// 0x4020,
	case XDI_FreeAcceleration		:// 0x4030,
	case XDI_AccelerationHR			:// 0x4040,
		return new XsVector3Variant(id);

	//case XDI_PositionGroup		:// 0x5000,
	case XDI_AltitudeMsl			:// 0x5010,
	case XDI_AltitudeEllipsoid		:// 0x5020,
		return new SimpleVariant<double>(id);
	case XDI_PositionEcef			:// 0x5030,
		return new XsVector3Variant(id);
	case XDI_LatLon					:// 0x5040,
		return new XsVector2Variant(id);

	//case XDI_SnapshotGroup		:// 0xC800,
	//case XDI_RetransmissionMask	:// 0x0001,
	//case XDI_RetransmissionFlag	:// 0x0001,
	case XDI_AwindaSnapshot 		:// 0xC810,
		return new XsAwindaSnapshotVariant(id);
	case XDI_FullSnapshot 			:// 0xC820,
		return new XsFullSnapshotVariant(id);

	//case XDI_GnssGroup			:// 0x7000,
	case XDI_GnssPvtData			:// 0x7010,
		return new XsRawGnssPvtDataVariant(id);
	case XDI_GnssSatInfo			:// 0x7020,
		return new XsRawGnssSatInfoVariant(id);

	//case XDI_AngularVelocityGroup	:// 0x8000,
	case XDI_RateOfTurn				:// 0x8020,
	case XDI_RateOfTurnHR			:// 0x8040,
		return new XsVector3Variant(id);
	case XDI_DeltaQ					:// 0x8030,
		return new XsQuaternionVariant(id);

	//case XDI_RawSensorGroup		:// 0xA000,
	//case XDI_RawUnsigned			:// 0x0000, //!< Tracker produces unsigned raw values, usually fixed behavior
	//case XDI_RawSigned			:// 0x0001, //!< Tracker produces signed raw values, usually fixed behavior
	case XDI_RawAccGyrMagTemp		:// 0xA010,
		return new XsScrDataVariant(id);

	case XDI_RawGyroTemp			:// 0xA020,
	case XDI_RawAcc					:// 0xA030,
	case XDI_RawGyr					:// 0xA040,
	case XDI_RawMag					:// 0xA050,
		return new XsUShortVectorVariant(id);

	case XDI_RawDeltaQ				:// 0xA060,
		return new XsQuaternionVariant(id);
	case XDI_RawDeltaV				:// 0xA070,
		return new XsVector3Variant(id);

	//case XDI_AnalogInGroup		:// 0xB000,
	case XDI_AnalogIn1				:// 0xB010,
	case XDI_AnalogIn2				:// 0xB020,
		return new SimpleVariant<uint16_t>(id);

	//case XDI_MagneticGroup		:// 0xC000,
	case XDI_MagneticField			:// 0xC020,
	case XDI_MagneticFieldCorrected :// 0xC040,

	//case XDI_VelocityGroup		:// 0xD000,
	case XDI_VelocityXYZ			:// 0xD010,
		return new XsVector3Variant(id);

	//case XDI_StatusGroup			:// 0xE000,
	case XDI_StatusByte				:// 0xE010,
		return new SimpleVariant<uint8_t>(id);
	case XDI_StatusWord				:// 0xE020,
		return new SimpleVariant<uint32_t>(id);
	case XDI_Rssi					:// 0xE040,
		return new SimpleVariant<uint8_t>(id);
	case XDI_DeviceId				:// 0xE080,
		return new SimpleVariant<uint32_t>(id);
	case XDI_LocationId				:// 0xE090
		return new SimpleVariant<uint16_t>(id);

	//case XDI_IndicationGroup		:// 0x4800, // 0100.1000 -> bit reverse = 0001.0010 -> type 18
	case XDI_TriggerIn1				:// 0x4810,
	case XDI_TriggerIn2				:// 0x4820,
		return new XsTriggerIndicationDataVariant(id);

	case XDI_RawBlob				:// 0xA080
		return new XsByteArrayVariant(id);

	case XDI_GloveSnapshot:			// 0xC830
		return new XsGloveSnapshotVariant(id);
	case XDI_GloveData:				// 0xC840
		return new XsGloveDataVariant(id);

	default:
		//JLERRORG("Unknown id: " << id);
		assert(0);
		return nullptr;
	}
}

XsUShortVector* rawVector(const XsDataPacket* thisPtr, XsUShortVector* returnVal, XsDataIdentifier id, XsUShortVector XsScrData::* field)
{
	assert(returnVal);
	auto it = MAP.find(XDI_RawAccGyrMagTemp);
	if (it != MAP.end())
		*returnVal = it->second->toDerived<XsScrDataVariant>().m_data.*field;
	else
	{
		it = MAP.find(id);
		if (it != MAP.end())
			*returnVal = it->second->toDerived<XsUShortVectorVariant>().m_data;
	}
	return returnVal;
}

void setRawVector(XsDataPacket* thisPtr, const XsUShortVector* vec, XsDataIdentifier id, XsUShortVector XsScrData::* field)
{
	detach(thisPtr);
	auto it = MAP.find(XDI_RawAccGyrMagTemp);
	if (it != MAP.end())
		it->second->toDerived<XsScrDataVariant>().m_data.*field = *vec;
	else
	{
		it = MAP.find(id);
		if (it != MAP.end())
			it->second->toDerived<XsUShortVectorVariant>().m_data = *vec;
		else
			MAP.insert(id, new XsUShortVectorVariant(id, *vec));
	}
}

template <typename T, typename V>
T* genericGet(const XsDataPacket* thisPtr, T* returnVal, XsDataIdentifier id, T const& failValue = T())
{
	assert(returnVal);
	auto it = MAP.find(id);
	if (it != MAP.end())
		*returnVal = it->second->toDerived<V>().m_data;
	else
		*returnVal = failValue;
	return returnVal;
}

template <typename T, typename V>
void genericSet(XsDataPacket* thisPtr, T const* val, XsDataIdentifier id)
{
	detach(thisPtr);
	assert(val);
	auto it = MAP.find(id);
	if (it != MAP.end())
	{
		it->second->toDerived<V>().m_data = *val;
		it->second->setDataId(id);
	}
	else
		MAP.insert(id, new V(id, *val));
}

template <typename T, typename V = SimpleVariant<T>>
struct GenericSimple {
static T get(const XsDataPacket* thisPtr, XsDataIdentifier id, T const& failValue = T())
{
	auto it = MAP.find(id);
	if (it != MAP.end())
		return it->second->toDerived<V>().m_data;
	return failValue;
}

static void set(XsDataPacket* thisPtr, T val, XsDataIdentifier id)
{
	detach(thisPtr);
	auto it = MAP.find(id);
	if (it != MAP.end())
		it->second->toDerived<V>().m_data = val;
	else
		MAP.insert(id, new V(id, val));
}
};

inline bool genericContains(const XsDataPacket* thisPtr, XsDataIdentifier id)
{
	return MAP.find(id) != MAP.end();
}

/*! \cond XS_INTERNAL */
/*!	\relates XsDataPacket
	\brief Check if data item contains quaternion orientation data
	\returns true if this packet contains quaternion orientation data
*/
int XsDataPacket_containsOrientationQuaternion(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_Quaternion);
}

/*! \relates XsDataPacket
	\brief Check if data item contains euler orientation data
	\returns true if this packet contains euler orientation data
*/
int XsDataPacket_containsOrientationEuler(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_EulerAngles);
}

/*! \relates XsDataPacket
	\brief Check if data item contains matrix orientation data
	\returns true if this packet contains matrix orientation data
*/
int XsDataPacket_containsOrientationMatrix(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_RotationMatrix);
}

/*! \brief Returns quaternion that defines the rotation necessary to rotate from coordinate system
	\a actual to \a desired when used as the left side in a quaternion multiplication
*/
XsQuaternion preRotFromXdi(XsDataIdentifier actual, XsDataIdentifier desired)
{
	static const XsQuaternion q_id(1, 0, 0, 0);										// x -> x
	static const XsQuaternion q_nwu2ned(0, 1, 0, 0);									// nwu -> ned = 180 degrees x
	static const XsQuaternion q_enu2nwu(1.4142135623730950488016887242097*0.5, 0, 0, -1.4142135623730950488016887242097*0.5);	// enu -> nwu = 90 degrees z
	static const XsQuaternion q_enu2ned(0, -1.4142135623730950488016887242097*0.5, -1.4142135623730950488016887242097*0.5, 0);	// enu -> ned = 90 degrees z followed by 180 degrees x

	static const XsQuaternion q_ned2nwu(0, -1, 0, 0);
	static const XsQuaternion q_nwu2enu(1.4142135623730950488016887242097*0.5, 0, 0, 1.4142135623730950488016887242097*0.5);
	static const XsQuaternion q_ned2enu(0, 1.4142135623730950488016887242097*0.5, 1.4142135623730950488016887242097*0.5, 0);

	switch (desired & XDI_CoordSysMask)
	{
	default:
	case XDI_CoordSysEnu:
		switch (actual & XDI_CoordSysMask)
		{
		default:
		case XDI_CoordSysEnu:
			return q_id;

		case XDI_CoordSysNed:
			return q_ned2enu;

		case XDI_CoordSysNwu:
			return q_nwu2enu;
		}

	case XDI_CoordSysNed:
		switch (actual & XDI_CoordSysMask)
		{
		default:
		case XDI_CoordSysEnu:
			return q_enu2ned;

		case XDI_CoordSysNed:
			return q_id;

		case XDI_CoordSysNwu:
			return q_nwu2ned;
		}

	case XDI_CoordSysNwu:
		switch (actual & XDI_CoordSysMask)
		{
		default:
		case XDI_CoordSysEnu:
			return q_enu2nwu;

		case XDI_CoordSysNed:
			return q_ned2nwu;

		case XDI_CoordSysNwu:
			return q_id;
		}
	}
}

/*! \endcond */


/*! \class XsDataPacket
	\brief Contains an interpreted data message. The class provides easy access to the contained
	data through its many functions.
	\sa cinterface For the C interface functions.
*/

/*! \addtogroup cinterface C Interface
	@{
*/

extern "C" {

/*! \brief Inits a data packet, the packet will be empty after construction
*/
void XsDataPacket_construct(XsDataPacket* thisPtr)
{
	thisPtr->d = new DataPacketPrivate;
	thisPtr->m_deviceId = 0;
	thisPtr->m_toa = 0;
	thisPtr->m_packetId = -1;
	thisPtr->m_etos = 0;
}

/*! \brief Inits a data packet as a (referenced) copy of \a src
*/
void XsDataPacket_copyConstruct(XsDataPacket* thisPtr, XsDataPacket const* src)
{
	++src->d->m_refCount;
	thisPtr->d = src->d;
	thisPtr->m_deviceId = src->m_deviceId;
	thisPtr->m_toa = src->m_toa;
	thisPtr->m_packetId = src->m_packetId;
	thisPtr->m_etos = src->m_etos;
}

/*! \brief Clears and frees data in an XsDataPacket
*/
void XsDataPacket_destruct(XsDataPacket* thisPtr)
{
	if (thisPtr->d && --thisPtr->d->m_refCount == 0)
		delete thisPtr->d;
	thisPtr->d = nullptr;
}

/*! \brief Clears all data in an XsDataPacket
	\param id The id to clear, supply XDI_None to clear the entire object
*/
void XsDataPacket_clear(XsDataPacket* thisPtr, XsDataIdentifier id)
{
	detach(thisPtr);
	if (id == XDI_None)
	{
		XsDataPacket_destruct(thisPtr);
		XsDataPacket_construct(thisPtr);
	}
	else
		MAP.erase(id);
}

/*! \brief Copy the XsDataPacket to \a copy
	\param copy The object to copy to
	\param src The source to copy from
*/
void XsDataPacket_copy(XsDataPacket* copy, XsDataPacket const* src)
{
	if (copy->d != src->d)
	{
		++src->d->m_refCount;
		if (--copy->d->m_refCount == 0)
			delete copy->d;
		copy->d = src->d;
	}
	copy->m_deviceId = src->m_deviceId;
	copy->m_toa = src->m_toa;
	copy->m_packetId = src->m_packetId;
	copy->m_etos = src->m_etos;
}

/*! \brief Swaps the XsDataPackets in \a thisPtr and \a other
	\param other The object to swap with
*/
void XsDataPacket_swap(XsDataPacket* thisPtr, XsDataPacket* other)
{
	std::swap(thisPtr->d, other->d);
	std::swap(thisPtr->m_deviceId, other->m_deviceId);
	std::swap(thisPtr->m_toa, other->m_toa);
	std::swap(thisPtr->m_packetId, other->m_packetId);
	std::swap(thisPtr->m_etos, other->m_etos);
}

/*! \brief Returns whether the datapacket is empty
	\return True when the XsDataPacket is empty
*/
int XsDataPacket_empty(const XsDataPacket* thisPtr)
{
	return 0 != MAP.empty();
}

/*! \brief Returns the number of individual items in the XsDataPacket
	\return The number of individual items in the XsDataPacket
*/
int XsDataPacket_itemCount(const XsDataPacket* thisPtr)
{
	assert(thisPtr);
	return (int) MAP.size();
}

/*!	\brief Returns the dataformat of a specific data identifier in the packet

	\param id : The XsDataIdentifier to query
	\returns Returns XDI_None if the packet does not contain the dataidentifier, the data
	format otherwise

	\sa XsDataIdentifier
*/
XsDataIdentifier XsDataPacket_dataFormat(const XsDataPacket* thisPtr, XsDataIdentifier id)
{
	auto it = MAP.find(id);
	if (it == MAP.end())
		return XDI_None;
	return it->second->dataId() & XDI_SubFormatMask;
}

/*! \brief helper for XsDataPacket_convertRawVector */
static XsReal signed_cast(uint16_t v)
{
	return (int16_t)v;
}

/*! \brief helper for XsDataPacket_convertRawVector */
static XsReal unsigned_cast(uint16_t v)
{
	return v;
}

/*!	\brief The raw accelerometer component of a data item.

	\param returnVal : An XsUShortVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector* XsDataPacket_rawAcceleration(const XsDataPacket* thisPtr, XsUShortVector* returnVal)
{
	return rawVector(thisPtr, returnVal, XDI_RawAcc, &XsScrData::m_acc);
}

/*! \brief Check if data item contains Raw Accelerometer data
	\returns true if this packet contains raw acceleration data
*/
int XsDataPacket_containsRawAcceleration(const XsDataPacket* thisPtr)
{
	return	MAP.find(XDI_RawAccGyrMagTemp) != MAP.end() ||
			MAP.find(XDI_RawAcc) != MAP.end();
}

/*! \brief Add/update raw accelerometer data for the item

	\param vec : The data to update the XsDataPacket with

	\details This will add the raw acceleration from \a vec to the data packet. If
			 the packet already contains raw acceleration, it will be replaced.
*/
void XsDataPacket_setRawAcceleration(XsDataPacket* thisPtr, const XsUShortVector* vec)
{
	setRawVector(thisPtr, vec, XDI_RawAcc, &XsScrData::m_acc);
}

/*! \brief The raw gyroscope component of a data item.

	\param returnVal : An XsUShortVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector* XsDataPacket_rawGyroscopeData(const XsDataPacket* thisPtr, XsUShortVector* returnVal)
{
	return rawVector(thisPtr, returnVal, XDI_RawGyr, &XsScrData::m_gyr);
}

/*! \brief Check if data item contains raw gyroscope data
	\returns true if this packet contains raw gyroscope data
*/
int XsDataPacket_containsRawGyroscopeData(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_RawAccGyrMagTemp) || genericContains(thisPtr, XDI_RawGyr);
}

/*! \brief Add/update raw gyroscope data for the item
	\param vec The new data to set
*/
void XsDataPacket_setRawGyroscopeData(XsDataPacket* thisPtr, const XsUShortVector* vec)
{
	setRawVector(thisPtr, vec, XDI_RawGyr, &XsScrData::m_gyr);
}

/*! \brief The raw magnetometer component of a data item.

	\param returnVal : An XsUShortVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector* XsDataPacket_rawMagneticField(const XsDataPacket* thisPtr, XsUShortVector* returnVal)
{
	return rawVector(thisPtr, returnVal, XDI_RawMag, &XsScrData::m_mag);
}

/*! \brief Check if data item contains raw magnetometer data
  \returns true if this packet contains raw magnetometer data
*/
int XsDataPacket_containsRawMagneticField(const XsDataPacket* thisPtr)
{
	return	MAP.find(XDI_RawAccGyrMagTemp) != MAP.end() ||
			MAP.find(XDI_RawMag) != MAP.end();
}

/*! \brief Add/update raw magnetometer data for the item
	\param vec The new data to set
*/
void XsDataPacket_setRawMagneticField(XsDataPacket* thisPtr, const XsUShortVector* vec)
{
	setRawVector(thisPtr, vec, XDI_RawMag, &XsScrData::m_mag);
}

/*! \brief The raw temperature component of a data item.

	\returns An uint16_t containing the raw temperature value
*/
uint16_t XsDataPacket_rawTemperature(const XsDataPacket* thisPtr)
{
	auto it = MAP.find(XDI_RawAccGyrMagTemp);
	if (it != MAP.end())
		return it->second->toDerived<XsScrDataVariant>().m_data.m_temp;
	else
		return 0;
}

/*! \brief Check if data item contains raw temperature data
  \returns true if this packet contains raw temperature data
*/
int XsDataPacket_containsRawTemperature(const XsDataPacket* thisPtr)
{
	return	MAP.find(XDI_RawAccGyrMagTemp) != MAP.end();
}

/*! \brief Add/update raw temperature data for the item
	\param temp The new data to set
*/
void XsDataPacket_setRawTemperature(XsDataPacket* thisPtr, uint16_t temp)
{
	detach(thisPtr);
	auto it = MAP.find(XDI_RawAccGyrMagTemp);
	if (it != MAP.end())
	{
		it->second->toDerived<XsScrDataVariant>().m_data.m_temp = temp;
	}
	else
	{
		auto v = new XsScrDataVariant(XDI_RawAccGyrMagTemp);
		v->m_data.m_temp = temp;
		MAP.insert(XDI_RawAccGyrMagTemp, v);
	}
}

/*! \brief The raw gyroscope temperature component of a data item.

	\param returnVal : An XsUShortVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector* XsDataPacket_rawGyroscopeTemperatureData(const XsDataPacket* thisPtr, XsUShortVector* returnVal)
{
	assert(returnVal);
	auto it = MAP.find(XDI_RawGyroTemp);
	if (it != MAP.end())
		*returnVal = it->second->toDerived<XsUShortVectorVariant>().m_data;
	return returnVal;
}

/*! \brief Check if data item contains raw gyroscope temperature data
  \returns true if this packet contains raw gyroscope temperature data
*/
int XsDataPacket_containsRawGyroscopeTemperatureData(const XsDataPacket* thisPtr)
{
	return MAP.find(XDI_RawGyroTemp) != MAP.end();
}

/*! \brief Add/update raw gyroscope temperature data for the item
	\param vec The new data to set
*/
void XsDataPacket_setRawGyroscopeTemperatureData(XsDataPacket* thisPtr, const XsUShortVector* vec)
{
	detach(thisPtr);
	auto it = MAP.find(XDI_RawGyroTemp);
	if (it != MAP.end())
		it->second->toDerived<XsUShortVectorVariant>().m_data = *vec;
	else
	{
		auto v = new XsUShortVectorVariant(XDI_RawGyroTemp);
		v->m_data = *vec;
		MAP.insert(XDI_RawGyroTemp, v);
	}
}

/*! \brief Return the raw data component of a data item.
	\param returnVal The object to store the requested data in
	\return The raw data component of a data item.
*/
XsScrData* XsDataPacket_rawData(const XsDataPacket* thisPtr, XsScrData* returnVal)
{
	assert(returnVal);
	auto it = MAP.find(XDI_RawAccGyrMagTemp);
	if (it != MAP.end())
		*returnVal = it->second->toDerived<XsScrDataVariant>().m_data;
	else
	{
		for (XsSize i = 0; i < 3; ++i)
		{
			returnVal->m_acc[i] = 0;
			returnVal->m_gyr[i] = 0;
			returnVal->m_mag[i] = 0;
		}
		returnVal->m_temp = 0;
	}
	return returnVal;	// not found
}

/*! \brief Check if data item contains raw data
  \returns true if this packet contains raw data
*/
int XsDataPacket_containsRawData(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_RawAccGyrMagTemp);
}

/*! \brief Add/update raw data for the item
	\param data The new data to set
*/
void XsDataPacket_setRawData(XsDataPacket* thisPtr, const XsScrData* data)
{
	detach(thisPtr);
	auto it = MAP.find(XDI_RawAccGyrMagTemp);
	if (it != MAP.end())
		it->second->toDerived<XsScrDataVariant>().m_data = *data;
	else
	{
		auto v = new XsScrDataVariant(XDI_RawAccGyrMagTemp);
		v->m_data = *data;
		MAP.insert(XDI_RawAccGyrMagTemp, v);
	}
}

/*! \brief The  delta velocity (deltaV) component of a data item.
\param returnVal : The XsVector that the deltaV will be assigned to
\returns An XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_velocityIncrement(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector3Variant>(thisPtr, returnVal, XDI_DeltaV);
}

/*! \brief Check if data item contains delta velocity data
\returns true if this packet contains delta velocity data
*/
int XsDataPacket_containsVelocityIncrement(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_DeltaV);
}

/*! \brief Add/update delta velocity data for the item
\param vec : The data to update the XsDataPacket with
*/
void XsDataPacket_setVelocityIncrement(XsDataPacket* thisPtr, const XsVector* vec)
{
	genericSet<XsVector, XsVector3Variant>(thisPtr, vec, XDI_DeltaV | XDI_SubFormatDouble);
}

/*! \brief The calibrated accelerometer component of a data item.

	\param returnVal : The XsVector that the calibrated acceleration will be assigned to

	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_calibratedAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector3Variant>(thisPtr, returnVal, XDI_Acceleration);
}

/*! \brief Check if data item contains calibrated accelerometer data
  \returns true if this packet contains calibrated accelerometer data
*/
int XsDataPacket_containsCalibratedAcceleration(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_Acceleration);
}

/*! \brief Add/update calibrated accelerometer data for the item
	\param vec : The data to update the XsDataPacket with
*/
void XsDataPacket_setCalibratedAcceleration(XsDataPacket* thisPtr, const XsVector* vec)
{
	genericSet<XsVector, XsVector3Variant>(thisPtr, vec, XDI_Acceleration | XDI_SubFormatDouble);
}


/*! \brief The  delta quaternion (deltaQ) component of a data item.
\param returnVal : The XsQuaternion that the deltaQ will be assigned to
\returns An XsQuaternion containing the deltaQ value
*/
XsQuaternion* XsDataPacket_orientationIncrement(const XsDataPacket* thisPtr, XsQuaternion* returnVal)
{
	return genericGet<XsQuaternion, XsQuaternionVariant>(thisPtr, returnVal, XDI_DeltaQ);
}

/*! \brief Check if data item contains delta quaternion data
\returns true if this packet contains delta quaternion data
*/
int XsDataPacket_containsOrientationIncrement(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_DeltaQ);
}

/*! \brief Add/update delta quaternion data for the item
\param quat : The data to update the XsDataPacket with
*/
void XsDataPacket_setOrientationIncrement(XsDataPacket* thisPtr, const XsQuaternion* quat)
{
	genericSet<XsQuaternion, XsQuaternionVariant>(thisPtr, quat, XDI_DeltaQ | XDI_SubFormatDouble);
}

/*! \brief The calibrated gyroscope component of a data item.

	\param returnVal : An XsVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_calibratedGyroscopeData(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector3Variant>(thisPtr, returnVal, XDI_RateOfTurn);
}

/*! \brief Check if data item contains calibrated gyroscope data
  \returns true if this packet contains calibrated gyroscope data
*/
int XsDataPacket_containsCalibratedGyroscopeData(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_RateOfTurn);
}

/*! \brief Add/update calibrated gyroscope data for the item
	\param vec : The data to update the XsDataPacket with
*/
void XsDataPacket_setCalibratedGyroscopeData(XsDataPacket* thisPtr, const XsVector* vec)
{
	genericSet<XsVector, XsVector3Variant>(thisPtr, vec, XDI_RateOfTurn | XDI_SubFormatDouble);
}

/*! \brief The calibrated magnetometer component of a data item.

	\param returnVal : An XsVector to put the requested in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_calibratedMagneticField(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector3Variant>(thisPtr, returnVal, XDI_MagneticField);
}

/*! \brief Check if data item contains calibrated magnetometer data
  \returns true if this packet contains calibrated magnetometer data
*/
int XsDataPacket_containsCalibratedMagneticField(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_MagneticField);
}

/*! \brief Add/update calibrated magnetometer data for the item
	\param vec : The data to update the XsDataPacket with
*/
void XsDataPacket_setCalibratedMagneticField(XsDataPacket* thisPtr, const XsVector* vec)
{
	genericSet<XsVector, XsVector3Variant>(thisPtr, vec, XDI_MagneticField | XDI_SubFormatDouble);
}

/*! \brief Return the calibrated Data component of a data item.
	\param returnVal Storage for the requested data
	\returns Returns the supplied \a returnVal filled with the requested data
*/
XsCalibratedData* XsDataPacket_calibratedData(const XsDataPacket* thisPtr, XsCalibratedData* returnVal)
{
	assert(returnVal);

	XsDataPacket_calibratedAcceleration(thisPtr, &returnVal->m_acc);
	XsDataPacket_calibratedGyroscopeData(thisPtr, &returnVal->m_gyr);
	XsDataPacket_calibratedMagneticField(thisPtr, &returnVal->m_mag);

	return returnVal;
}

/*! \brief Check if data item contains calibrated Data
	\returns Returns whether the packet contains calibrated data or not
	\note Calibrated data is only present if *all* components are present.
*/
int XsDataPacket_containsCalibratedData(const XsDataPacket* thisPtr)
{
	// Note: calibrated data is only present if *all* components are present
	return	genericContains(thisPtr, XDI_Acceleration) &&
			genericContains(thisPtr, XDI_RateOfTurn) &&
			genericContains(thisPtr, XDI_MagneticField);
}

/*! \brief Add/update calibrated Data for the item
	\param data : The data to update the XsDataPacket with
 */
void XsDataPacket_setCalibratedData(XsDataPacket* thisPtr, const XsCalibratedData* data)
{
	XsDataPacket_setCalibratedAcceleration(thisPtr, &data->m_acc);
	XsDataPacket_setCalibratedGyroscopeData(thisPtr, &data->m_gyr);
	XsDataPacket_setCalibratedMagneticField(thisPtr, &data->m_mag);
}

/*! \brief The corrected magnetometer component of a data item (ICC result).

\param returnVal : An XsVector to put the requested in

\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_correctedMagneticField(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector3Variant>(thisPtr, returnVal, XDI_MagneticFieldCorrected);
}

/*! \brief Check if data item contains corrected magnetometer data (ICC result).
\returns true if this packet contains corrected magnetometer data
*/
int XsDataPacket_containsCorrectedMagneticField(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_MagneticFieldCorrected);
}

/*! \brief Add/update corrected magnetometer data for the item (ICC result).
\param vec : The data to update the XsDataPacket with
*/
void XsDataPacket_setCorrectedMagneticField(XsDataPacket* thisPtr, const XsVector* vec)
{
	genericSet<XsVector, XsVector3Variant>(thisPtr, vec, XDI_MagneticFieldCorrected | XDI_SubFormatDouble);
}

/*! \brief Return the orientation component of a data item as a quaternion.

	\param returnVal An %XsQuaternion to put the requested orientation in
	\param coordinateSystem The coordinate system of the requested orientation. If this does not match
	the stored coordinate system, it will be transformed to the requested orientation.

	\returns An XsQuaternion containing the orientation data
*/
XsQuaternion* XsDataPacket_orientationQuaternion(const XsDataPacket* thisPtr, XsQuaternion* returnVal, XsDataIdentifier coordinateSystem)
{
	assert(returnVal);
	auto it = MAP.find(XDI_Quaternion);
	if (it != MAP.end())
	{
		*returnVal = it->second->toDerived<XsQuaternionVariant>().m_data;
		XsDataIdentifier foundId = it->second->dataId();
		if ((coordinateSystem & XDI_CoordSysMask) != (foundId & XDI_CoordSysMask))
		{
			XsQuaternion rot;
			rot = preRotFromXdi(foundId, coordinateSystem);
			XsQuaternion_multiply(&rot, returnVal, returnVal);
		}
	}
	else if (XsDataPacket_containsOrientationMatrix(thisPtr))
	{
		XsMatrix3x3 m;
		XsDataPacket_orientationMatrix(thisPtr, &m, coordinateSystem);
		returnVal->fromRotationMatrix(m);
	}
	else if (XsDataPacket_containsOrientationEuler(thisPtr))
	{
		XsEuler eul;
		XsDataPacket_orientationEuler(thisPtr, &eul, coordinateSystem);
		returnVal->fromEulerAngles(eul);
	}
	//else
	//	memset(returnVal->m_data, 0, 4*sizeof(XsReal));

	return returnVal;
}

/*! \brief Removes all orientations from the datapacket */
static void removeAllOrientations(XsDataPacket* thisPtr)
{
	detach(thisPtr);
	MAP.erase(XDI_Quaternion);
	MAP.erase(XDI_RotationMatrix);
	MAP.erase(XDI_EulerAngles);
}

/*! \brief Add/update quaternion orientation Data for the item
	\param data The new data to set
	\param coordinateSystem The coordinate system of the requested orientation.
*/
void XsDataPacket_setOrientationQuaternion(XsDataPacket* thisPtr, const XsQuaternion* data, XsDataIdentifier coordinateSystem)
{
	// always create a new one
	removeAllOrientations(thisPtr);
	MAP.insert(XDI_Quaternion, new XsQuaternionVariant(XDI_Quaternion | XDI_SubFormatDouble | (coordinateSystem & XDI_CoordSysMask), *data));
}

/*! \brief Return the orientation component of a data item as a euler angles.

	\param returnVal An %XsEuler to put the requested orientation in
	\param coordinateSystem The coordinate system of the requested orientation. If this does not match
	the stored coordinate system, it will be transformed to the requested orientation.

	\returns A %XsEuler containing the orientation data
*/
XsEuler* XsDataPacket_orientationEuler(const XsDataPacket* thisPtr, XsEuler* returnVal, XsDataIdentifier coordinateSystem)
{
	assert(returnVal);
	auto it = MAP.find(XDI_EulerAngles);
	if (it != MAP.end())
	{
		*returnVal = it->second->toDerived<XsEulerVariant>().m_data;
		XsDataIdentifier foundId = it->second->dataId();
		if ((coordinateSystem & XDI_CoordSysMask) != (foundId & XDI_CoordSysMask))
		{
			XsQuaternion rot, q;
			rot = preRotFromXdi(foundId, coordinateSystem);
			XsQuaternion_fromEulerAngles(&q, returnVal);
			XsQuaternion_multiply(&rot, &q, &q);
			XsEuler_fromQuaternion(returnVal, &q);
		}
	}
	else if (XsDataPacket_containsOrientationMatrix(thisPtr))
	{
		XsMatrix3x3 m;
		XsDataPacket_orientationMatrix(thisPtr, &m, coordinateSystem);
		XsQuaternion q;
		q.fromRotationMatrix(m);
		returnVal->fromQuaternion(q);
	}
	else if (XsDataPacket_containsOrientationQuaternion(thisPtr))
	{
		XsQuaternion q;
		XsDataPacket_orientationQuaternion(thisPtr, &q, coordinateSystem);
		returnVal->fromQuaternion(q);
	}
	//else
	//{
	//	returnVal->m_x = XsMath_zero;
	//	returnVal->m_y = XsMath_zero;
	//	returnVal->m_z = XsMath_zero;
	//}

	return returnVal;
}

/*! \brief Add/update quaternion orientation Data for the item
	\param data The new data to set
	\param coordinateSystem The coordinate system of the orientation.
*/
void XsDataPacket_setOrientationEuler(XsDataPacket* thisPtr, const XsEuler* data, XsDataIdentifier coordinateSystem)
{
	// always create a new one
	removeAllOrientations(thisPtr);
	MAP.insert(XDI_EulerAngles, new XsEulerVariant(XDI_EulerAngles | XDI_SubFormatDouble | (coordinateSystem & XDI_CoordSysMask), *data));
}

/*! \brief Return the orientation component of a data item as a orientation matrix.

	\param returnVal An %XsMatrix to put the requested orientation in
	\param coordinateSystem The coordinate system of the requested orientation. If this does not match
	the stored coordinate system, it will be transformed to the requested orientation.

	\returns An %XsMatrix containing the orientation data
*/
XsMatrix* XsDataPacket_orientationMatrix(const XsDataPacket* thisPtr, XsMatrix* returnVal, XsDataIdentifier coordinateSystem)
{
	assert(returnVal);
	auto it = MAP.find(XDI_RotationMatrix);
	if (it != MAP.end())
	{
		*returnVal = it->second->toDerived<XsMatrixVariant>().m_data;
		XsDataIdentifier foundId = it->second->dataId();
		if ((coordinateSystem & XDI_CoordSysMask) != (foundId & XDI_CoordSysMask))
		{
			XsQuaternion rot, q;
			rot = preRotFromXdi(foundId, coordinateSystem);
			XsQuaternion_fromRotationMatrix(&q, returnVal);
			XsQuaternion_multiply(&rot, &q, &q);
			XsMatrix_fromQuaternion(returnVal, &q);
		}
	}
	else if (XsDataPacket_containsOrientationQuaternion(thisPtr))
	{
		XsQuaternion q;
		XsDataPacket_orientationQuaternion(thisPtr, &q, coordinateSystem);
		returnVal->fromQuaternion(q);
	}
	else if (XsDataPacket_containsOrientationEuler(thisPtr))
	{
		XsEuler eul;
		XsDataPacket_orientationEuler(thisPtr, &eul, coordinateSystem);
		XsQuaternion q;
		q.fromEulerAngles(eul);
		returnVal->fromQuaternion(q);
	}
	//else
	//	memset(returnVal->m_data, 0, 4*sizeof(XsReal));

	return returnVal;
}

/*! \brief Add/update quaternion orientation Data for the item
	\param data The new data to set
	\param coordinateSystem The coordinate system of the orientation.
*/
void XsDataPacket_setOrientationMatrix(XsDataPacket* thisPtr, const XsMatrix* data, XsDataIdentifier coordinateSystem)
{
	// always create a new one
	removeAllOrientations(thisPtr);
	MAP.insert(XDI_RotationMatrix, new XsMatrixVariant(XDI_RotationMatrix | XDI_SubFormatDouble | (coordinateSystem & XDI_CoordSysMask), *data));
}

/*! \brief Check if data item contains orientation Data of any kind
	\returns true if this packet contains orientation data
*/
int XsDataPacket_containsOrientation(const XsDataPacket* thisPtr)
{
	return	genericContains(thisPtr, XDI_Quaternion) ||
			genericContains(thisPtr, XDI_EulerAngles) ||
			genericContains(thisPtr, XDI_RotationMatrix);
}

/*! \brief Returns the data identifier of the first orientation data of any kind in the packet
	\returns The %XsDataIdentifier of the first orientation data of any kind in the packet
*/
XsDataIdentifier XsDataPacket_orientationIdentifier(const XsDataPacket* thisPtr)
{
	auto it = MAP.find(XDI_Quaternion);
	if (it != MAP.end())
		return it->second->dataId();

	it = MAP.find(XDI_EulerAngles);
	if (it != MAP.end())
		return it->second->dataId();

	it = MAP.find(XDI_RotationMatrix);
	if (it != MAP.end())
		return it->second->dataId();

	return XDI_None;
}

/*! \brief Returns the coordinate system of the first orientation data of any kind in the packet
	\returns The XsDataIdentifier of the coordinate system of the first orientation data of any kind in the packet
*/
XsDataIdentifier XsDataPacket_coordinateSystemOrientation(const XsDataPacket* thisPtr)
{
	return XsDataPacket_orientationIdentifier(thisPtr) & XDI_CoordSysMask;
}

/*! \brief Check if data item contains pressure data
  \returns true if this packet contains pressure data
*/
int XsDataPacket_containsPressure(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_BaroPressure);
}

/*! \brief Check if data item contains pressure age data
  \returns true if this packet contains pressure age data
*/
int XsDataPacket_containsPressureAge(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_PressureAge);
}

/*! \brief The air pressure component of a data item.

	\param returnVal : An XsPressure object to put the requested in

	\returns An XsPressure object containing the pressure and if available the pressure age
*/
XsPressure* XsDataPacket_pressure(const XsDataPacket* thisPtr, XsPressure* returnVal)
{
	assert(returnVal);
	auto it = MAP.find(XDI_BaroPressure);
	if (it != MAP.end())
	{
		returnVal->m_pressure = it->second->toDerived<SimpleVariant<uint32_t>>().m_data;
		returnVal->m_pressureAge = 0;
	}

	it = MAP.find(XDI_PressureAge);
	if (it != MAP.end())
		returnVal->m_pressureAge = it->second->toDerived<SimpleVariant<uint8_t>>().m_data;

	return returnVal;
}

/*! \brief Add/update pressure data for the item
	\param data The new data to set
*/
void XsDataPacket_setPressure(XsDataPacket* thisPtr, const XsPressure* data)
{
	GenericSimple<uint32_t>::set(thisPtr, (uint32_t) XsMath::doubleToLong(data->m_pressure), XDI_BaroPressure);
	GenericSimple<uint8_t>::set(thisPtr, data->m_pressureAge, XDI_PressureAge);
}

/*! \brief Return the strapdown integration data component of a data item.
	\param returnVal Storage for the requested data
	\returns Returns the supplied \a returnVal filled with the requested data
*/
XsSdiData* XsDataPacket_sdiData(const XsDataPacket* thisPtr, XsSdiData* returnVal)
{
	assert(returnVal);
	auto it = MAP.find(XDI_DeltaQ);
	if (it != MAP.end())
		returnVal->setOrientationIncrement(it->second->toDerived<XsQuaternionVariant>().m_data);

	it = MAP.find(XDI_DeltaV);
	if (it != MAP.end())
		returnVal->setVelocityIncrement(it->second->toDerived<XsVector3Variant>().m_data);

	return returnVal;
}

/*! \brief Check if data item contains strapdown integration data
	\returns Returns true if this packet contains sdi data
*/
int XsDataPacket_containsSdiData(const XsDataPacket* thisPtr)
{
	return	genericContains(thisPtr, XDI_DeltaQ) &&
			genericContains(thisPtr, XDI_DeltaV);
}

/*! \brief Add/update strapdown integration data for the item
	\param data The updated data
*/
void XsDataPacket_setSdiData(XsDataPacket* thisPtr, const XsSdiData* data)
{
	genericSet<XsQuaternion, XsQuaternionVariant>(thisPtr, &data->orientationIncrement(), XDI_DeltaQ | XDI_SubFormatDouble);
	genericSet<XsVector3, XsVector3Variant>(thisPtr, &data->velocityIncrement(), XDI_DeltaV | XDI_SubFormatDouble);
}

/*! \brief Return the glove data component of a data item.
\param returnVal Storage for the requested data
\returns Returns the supplied \a returnVal filled with the requested data
*/
XsGloveData* XsDataPacket_gloveData(const XsDataPacket* thisPtr, XsGloveData* returnVal)
{
	return genericGet<XsGloveData, XsGloveDataVariant>(thisPtr, returnVal, XDI_GloveData);
}

/*! \brief Check if data item contains glove data
\returns Returns true if this packet contains sdi data
*/
int XsDataPacket_containsGloveData(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_GloveData);
}

/*! \brief Add/update strapdown integration data for the item
\param data The updated data
*/
void XsDataPacket_setGloveData(XsDataPacket* thisPtr, const XsGloveData* data)
{
	genericSet<XsGloveData, XsGloveDataVariant>(thisPtr, data, XDI_GloveData);
}

/*! \brief The device id of a data item.

	\param returnVal an XsDeviceId object to put the requested data in

	\returns An XsDeviceId object containing the requested device ID
*/
XsDeviceId* XsDataPacket_storedDeviceId(const XsDataPacket* thisPtr, XsDeviceId* returnVal)
{
	return genericGet<XsDeviceId, SimpleVariant<uint32_t>>(thisPtr, returnVal, XDI_DeviceId);
}

/*! \brief Return non-zero if this data packet stores a device ID
	\returns non-zero (true) if a device ID is available, zero (false) otherwise
*/
int XsDataPacket_containsStoredDeviceId(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_DeviceId);
}

/*! \brief Add or update device id for the item
	\param data the device ID to store in the packet
*/
void XsDataPacket_setStoredDeviceId(XsDataPacket* thisPtr, const XsDeviceId* data)
{
	GenericSimple<uint32_t>::set(thisPtr, data->legacyDeviceId(), XDI_DeviceId);
}

/*! \brief The location ID of a data item.

\returns The requested location ID
*/
uint16_t XsDataPacket_storedLocationId(const XsDataPacket* thisPtr)
{
	return GenericSimple<uint16_t>::get(thisPtr, XDI_LocationId);
}

/*! \brief Return non-zero if this data packet stores a location ID
\returns non-zero (true) if a location ID is available, zero (false) otherwise
*/
int XsDataPacket_containsStoredLocationId(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_LocationId);
}

/*! \brief Add or update location ID for the item
\param data the location ID to store in the packet
*/
void XsDataPacket_setStoredLocationId(XsDataPacket* thisPtr, uint16_t data)
{
	GenericSimple<uint16_t>::set(thisPtr, data, XDI_LocationId);
}

/*! \brief The temperature component of a data item.
	\returns A double containing the temperature value, -1000.0 if the packet does not contain temperature
*/
double XsDataPacket_temperature(const XsDataPacket* thisPtr)
{
	return GenericSimple<double>::get(thisPtr, XDI_Temperature);
}

/*! \brief Check if data item contains temperature data
  \returns true if this packet contains temperature data
*/
int XsDataPacket_containsTemperature(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_Temperature);
}

/*! \brief Adds or updates the temperature data in the datapacket
	\details The \a temp is added to the datapacket. If the packet already contains
			 temperature it is replaced with the new value.
	\param temperature : The temperature to set
*/
void XsDataPacket_setTemperature(XsDataPacket* thisPtr, double temperature)
{
	GenericSimple<double>::set(thisPtr, temperature, XDI_Temperature | XDI_SubFormatDouble);
}

/*! \brief The analog in 1 component of a data item.
	\param returnVal : The XsAnalogInData object that the analog in 1 value will be assigned to
	\returns An XsAnalogInData containing the analog in 1 value
*/
XsAnalogInData* XsDataPacket_analogIn1Data(const XsDataPacket* thisPtr, XsAnalogInData* returnVal)
{
	return genericGet<XsAnalogInData, SimpleVariant<uint16_t>>(thisPtr, returnVal, XDI_AnalogIn1);
}

/*! \brief Check if data item contains analog in 1 data
	\returns true if this packet contains analog in 1 data
*/
int XsDataPacket_containsAnalogIn1Data(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_AnalogIn1);
}

/*! \brief Add/update analog in 1 data for the item
	\param data The \a data is added to the datapacket. If the packet already contains
			analogin1 data, it is replaced with the new value
*/
void XsDataPacket_setAnalogIn1Data(XsDataPacket* thisPtr, const XsAnalogInData* data)
{
	GenericSimple<uint16_t>::set(thisPtr, data->m_data, XDI_AnalogIn1);
}

/*! \brief The analog in 2 component of a data item.
	\param returnVal : The XsAnalogInData object that the analog in 2 value will be assigned to
	\returns An XsAnalogInData containing the analog in 2 value
*/
XsAnalogInData* XsDataPacket_analogIn2Data(const XsDataPacket* thisPtr, XsAnalogInData* returnVal)
{
	return genericGet<XsAnalogInData, SimpleVariant<uint16_t>>(thisPtr, returnVal, XDI_AnalogIn2);
}

/*! \brief Check if data item contains analog in 2 data
  \returns true if this packet contains analog in 2 data
*/
int XsDataPacket_containsAnalogIn2Data(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_AnalogIn2);
}

/*! \brief Add/update analog in 2 data for the item
	\param data The \a data is added to the datapacket. If the packet already contains
			analogin1 data, it is replaced with the new value
*/
void XsDataPacket_setAnalogIn2Data(XsDataPacket* thisPtr, const XsAnalogInData* data)
{
	GenericSimple<uint16_t>::set(thisPtr, data->m_data, XDI_AnalogIn2);
}

/*! \brief The position lat lon alt component of a data item.

	\param returnVal : The XsVector to return the requested data in

	\returns An XsVector containing the latitude, longitude and altitude values in that order
*/
XsVector* XsDataPacket_positionLLA(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	assert(returnVal);
	returnVal->setSize(3);

	XsVector latlon(2);
	genericGet<XsVector, XsVector2Variant>(thisPtr, &latlon, XDI_LatLon);
	if (!latlon.empty())
	{
		(*returnVal)[0] = latlon[0];
		(*returnVal)[1] = latlon[1];
	}
	else
	{
		(*returnVal)[0] = 0;
		(*returnVal)[1] = 0;
	}
	// This triggers a warning when XsReal is float, which is deliberately not silenced
	(*returnVal)[2] = GenericSimple<double>::get(thisPtr, XDI_AltitudeEllipsoid);

	return returnVal;
}

/*! \brief Check if data item contains position lat lon alt data
	\returns true if this packet contains position lat lon alt data
*/
int XsDataPacket_containsPositionLLA(const XsDataPacket* thisPtr)
{
	return	genericContains(thisPtr, XDI_LatLon) &&
			genericContains(thisPtr, XDI_AltitudeEllipsoid);
}

/*! \brief Add/update position lat lon alt data for the item

	\param data : The XsVector that conrtains the Lat/Long/Alt data to store in the packet
*/
void XsDataPacket_setPositionLLA(XsDataPacket* thisPtr, const XsVector* data)
{
	XsVector latlon(2);
	latlon[0] = (*data)[0];
	latlon[1] = (*data)[1];

	genericSet<XsVector, XsVector2Variant>(thisPtr, &latlon, XDI_LatLon | XDI_SubFormatDouble);
	GenericSimple<double>::set(thisPtr, (*data)[2], XDI_AltitudeEllipsoid | XDI_SubFormatDouble);
}

/*! \brief The position latitude longitude component of a data item.

	\param returnVal : The XsVector to return the requested data in

	\returns An XsVector containing the latitude and longitude values in that order
	\sa XsDataPacket_containsLatitudeLongitude
	\sa XsDataPacket_positionLLA \sa XsDataPacket_altitude
*/
XsVector* XsDataPacket_latitudeLongitude(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector2Variant>(thisPtr, returnVal, XDI_LatLon);
}

/*! \brief Check if data item contains position latitude longitude data
	\returns true if this packet contains position latitude longitude data
	\sa XsDataPacket_containsPositionLLA \sa XsDataPacket_containsAltitude
*/
int XsDataPacket_containsLatitudeLongitude(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_LatLon);
}

/*! \brief Add/update position latitude longitude data for the item

	\param data : The XsVector that contains the latitude longitude data to store in the packet
	\sa XsDataPacket_setPositionLLA \sa XsDataPacket_setAltitude
*/
void XsDataPacket_setLatitudeLongitude(XsDataPacket* thisPtr, const XsVector* data)
{
	genericSet<XsVector, XsVector2Variant>(thisPtr, data, XDI_LatLon | XDI_SubFormatDouble);
}

/*! \brief The position altitude component of a data item.

	\returns The altitude stored in the packet or XsMath_infinity if no altitude is available
	\sa XsDataPacket_containsAltitude
	\sa XsDataPacket_positionLLA \sa XsDataPacket_latitudeLongitude
*/
double XsDataPacket_altitude(const XsDataPacket* thisPtr)
{
	return GenericSimple<double>::get(thisPtr, XDI_AltitudeEllipsoid);
}

/*! \brief Check if data item contains position altitude data
	\returns true if this packet contains position altitude data
	\sa XsDataPacket_containsPositionLLA \sa XsDataPacket_containsLatitudeLongitude
*/
int XsDataPacket_containsAltitude(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_AltitudeEllipsoid);
}

/*! \brief Add/update altitude data for the item
	\param data : The altitude data to store in the packet
	\sa XsDataPacket_setPositionLLA \sa XsDataPacket_setLatitudeLongitude
*/
void XsDataPacket_setAltitude(XsDataPacket* thisPtr, double data)
{
	GenericSimple<double>::set(thisPtr, data, XDI_AltitudeEllipsoid | XDI_SubFormatDouble);
}

/*! \brief The position altitude above MSL component of a data item.

	\returns The altitude stored in the packet or XsMath_infinity if no altitude is available
	\sa XsDataPacket_containsAltitudeMsl
*/
double XsDataPacket_altitudeMsl(const XsDataPacket* thisPtr)
{
	return GenericSimple<double>::get(thisPtr, XDI_AltitudeMsl);
}

/*! \brief Check if data item contains position altitude above MSL data

	\returns true if this packet contains position altitude data
*/
int XsDataPacket_containsAltitudeMsl(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_AltitudeMsl);
}

/*! \brief Add/update altitude above MSL data for the item

	\param data : The altitude data to store in the packet
*/
void XsDataPacket_setAltitudeMsl(XsDataPacket* thisPtr, double data)
{
	GenericSimple<double>::set(thisPtr, data, XDI_AltitudeMsl | XDI_SubFormatDouble);
}

/*! \brief  The velocity NWU component of a data item.

	\param returnVal : The XsVector to put the data in
	\param coordinateSystem The coordinate system of the requested velocity. If this does not match
	the stored coordinate system, it will be transformed to the requested velocity.
	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_velocity(const XsDataPacket* thisPtr, XsVector* returnVal, XsDataIdentifier coordinateSystem)
{
	assert(returnVal);
	auto it = MAP.find(XDI_VelocityXYZ);
	if (it != MAP.end())
	{
		*returnVal = it->second->toDerived<XsVector3Variant>().m_data;
		XsDataIdentifier actualCsys = (it->second->dataId() & XDI_CoordSysMask);
		XsDataIdentifier desiredCsys = (coordinateSystem & XDI_CoordSysMask);
		if (actualCsys != desiredCsys)
		{
			XsVector3 vel(*returnVal);
			XsVector& rv = *returnVal;

			switch (desiredCsys)
			{
				case XDI_CoordSysEnu:
				{
					if (actualCsys == XDI_CoordSysNwu)
					{
						rv[0] = -vel[1];
						rv[1] = vel[0];
						rv[2] = vel[2];
					}
					else if (actualCsys == XDI_CoordSysNed)
					{
						rv[0] = vel[1];
						rv[1] = vel[0];
						rv[2] = -vel[2];
					}
				} break;

				case XDI_CoordSysNwu:
				{
					if (actualCsys == XDI_CoordSysEnu)
					{
						rv[0] = vel[1];
						rv[1] = -vel[0];
						rv[2] = vel[2];
					}
					else if (actualCsys == XDI_CoordSysNed)
					{
						rv[0] = vel[0];
						rv[1] = -vel[1];
						rv[2] = -vel[2];
					}
				} break;

				case XDI_CoordSysNed:
				{
					if (actualCsys == XDI_CoordSysEnu)
					{
						rv[0] = vel[1];
						rv[1] = vel[0];
						rv[2] = -vel[2];
					}
					else if (actualCsys == XDI_CoordSysNwu)
					{
						rv[0] = vel[0];
						rv[1] = -vel[1];
						rv[2] = -vel[2];
					}
				} break;

				default:
				{
					rv[0] = vel[0];
					rv[1] = vel[1];
					rv[2] = vel[2];
				} break;
			}
		}
	}

	return returnVal;
}

/*! \brief Check if data item contains velocity NED data
	\returns true if this packet contains velocity NED data
*/
int XsDataPacket_containsVelocity(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_VelocityXYZ);
}

/*! \brief Add/update velocity NED data for the item
	\param data The new data to set
	\param coordinateSystem The coordinate system of the requested orientation.
*/
void XsDataPacket_setVelocity(XsDataPacket* thisPtr, const XsVector* data, XsDataIdentifier coordinateSystem)
{
	detach(thisPtr);
	auto it = MAP.find(XDI_VelocityXYZ);
	if (it != MAP.end())
		MAP.erase(it);

	genericSet<XsVector, XsVector3Variant>(thisPtr, data, XDI_VelocityXYZ | XDI_SubFormatDouble | (coordinateSystem & XDI_CoordSysMask));
}

/*! \brief Returns the data identifier of the first velocity data of any kind in the packet
	\returns The %XsDataIdentifier of the first velocity data of any kind in the packet
*/
XsDataIdentifier XsDataPacket_velocityIdentifier(const XsDataPacket* thisPtr)
{
	auto it = MAP.find(XDI_VelocityXYZ);
	if (it != MAP.end())
		return it->second->dataId();
	return XDI_None;
}

/*! \brief Returns the coordinate system of the first velocity data of any kind in the packet
	\returns The XsDataIdentifier of the coordinate system of the first velocity data of any kind in the packet
*/
XsDataIdentifier XsDataPacket_coordinateSystemVelocity(const XsDataPacket* thisPtr)
{
	return XsDataPacket_velocityIdentifier(thisPtr) & XDI_CoordSysMask;
}

/*! \brief The status component of a data item.

	\returns An uint32_t containing the status value
*/
uint32_t XsDataPacket_status(const XsDataPacket* thisPtr)
{
	auto it = MAP.find(XDI_StatusWord);
	if (it != MAP.end())
		return it->second->toDerived<SimpleVariant<uint32_t>>().m_data;

	it = MAP.find(XDI_StatusByte);
	if (it != MAP.end())
		return it->second->toDerived<SimpleVariant<uint8_t>>().m_data;

	return 0;
}

/*! \brief Check if data item contains detailed status data
  \returns true if this packet contains detailed status data
*/
int XsDataPacket_containsDetailedStatus(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_StatusWord);
}

/*! \brief Check if data item contains legacy status data
  \returns true if this packet contains legacy status data
*/
int XsDataPacket_containsStatus(const XsDataPacket* thisPtr)
{
	return	genericContains(thisPtr, XDI_StatusWord) ||
			genericContains(thisPtr, XDI_StatusByte);
}

/*! \brief Add/update status data for the item
	\param data The new data to set
*/
void XsDataPacket_setStatusByte(XsDataPacket* thisPtr, uint8_t data)
{
	detach(thisPtr);
	if (genericContains(thisPtr, XDI_StatusWord))
	{
		uint32_t word = GenericSimple<uint32_t>::get(thisPtr, XDI_StatusWord);
		word = ((word & (~0xFF)) | data);
		XsDataPacket_setStatus(thisPtr, word);
	}
	else
		GenericSimple<uint8_t>::set(thisPtr, data, XDI_StatusByte);
}

/*! \brief Add/update status data for the item
	\param data The new data to set
*/
void XsDataPacket_setStatus(XsDataPacket* thisPtr, uint32_t data)
{
	detach(thisPtr);
	auto it = MAP.find(XDI_StatusByte);
	if (it != MAP.end())
		MAP.erase(it);

	GenericSimple<uint32_t>::set(thisPtr, data, XDI_StatusWord);
}

/*! \brief Returns the trigger indication data of a packet
	If the packet does not contain the requested data, the return val struct will be set to all zeroes
	\param[in] triggerId The trigger data identifier to add data for (e.g. XDI_TriggerIn1 or XDI_TriggerIn2)
	\param[out] returnVal pointer to the trigger indication data of a packet.
	\note returnVal should point to a buffer large enough to hold sizeof(XsTriggerIndicationData) bytes of data
	\returns Returns a pointer to the trigger indication data of a packet
*/
XsTriggerIndicationData* XsDataPacket_triggerIndication(const XsDataPacket* thisPtr, XsDataIdentifier triggerId, XsTriggerIndicationData* returnVal)
{
	return genericGet<XsTriggerIndicationData, XsTriggerIndicationDataVariant>(thisPtr, returnVal, triggerId);
}

/*! \brief Check if data item contains trigger indication data
	\param[in] triggerId The trigger data identifier to check (e.g. XDI_TriggerIn1 or XDI_TriggerIn2)
	\returns true if this packet contains trigger indication data
*/
int XsDataPacket_containsTriggerIndication(const XsDataPacket* thisPtr, XsDataIdentifier triggerId)
{
	return genericContains(thisPtr, triggerId);
}

/*! \brief Add/update trigger indication data for the item
	\param[in] triggerId The trigger data identifier to add data for (e.g. XDI_TriggerIn1 or XDI_TriggerIn2)
	\param[in] triggerIndicationData pointer the a XsTriggerIndicationData buffer containing the data to set
*/
void XsDataPacket_setTriggerIndication(XsDataPacket* thisPtr, XsDataIdentifier triggerId, const XsTriggerIndicationData *triggerIndicationData)
{
	genericSet<XsTriggerIndicationData, XsTriggerIndicationDataVariant>(thisPtr, triggerIndicationData, triggerId);
}

/*! \brief Return the 8 bit packet counter of a packet

	\details This function returns an 8 bit packet counter as used by some third party devices

	\returns Returns the 8 bit packet counter of a packet
*/
uint8_t XsDataPacket_packetCounter8(const XsDataPacket* thisPtr)
{
	return GenericSimple<uint8_t>::get(thisPtr, XDI_PacketCounter8);
}

/*! \brief Check if data item contains an 8 bit packet counter
	\returns true if this packet contains an 8 bit packet counter
*/
int XsDataPacket_containsPacketCounter8(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_PacketCounter8);
}

/*! \brief Add/update 8 bit packet counter data for the item
	\param counter The new data to set
*/
void XsDataPacket_setPacketCounter8(XsDataPacket* thisPtr, uint8_t counter)
{
	GenericSimple<uint8_t>::set(thisPtr, counter, XDI_PacketCounter8);
}

/*! \brief Return the packet/frame counter of a packet

	\details For strapdown integration data, this function will return the m_wlastFrameNumber
	For other data, this function will return the m_sc

	This way there is a function that will always return the counter of a packet

	\returns Returns the packet/frame counter of a packet
*/
uint16_t XsDataPacket_packetCounter(const XsDataPacket* thisPtr)
{
	auto it = MAP.find(XDI_PacketCounter);
	if (it != MAP.end())
		return it->second->toDerived<SimpleVariant<uint16_t>>().m_data;

	it = MAP.find(XDI_FrameRange);
	if (it != MAP.end())
		return static_cast<uint16_t>(static_cast<unsigned int>(it->second->toDerived<XsRangeVariant>().m_data.last()));

	return 0;
}

/*! \brief Check if data item contains a packet counter
	\returns true if this packet contains a packet counter
*/
int XsDataPacket_containsPacketCounter(const XsDataPacket* thisPtr)
{
	return	genericContains(thisPtr, XDI_PacketCounter) ||
			genericContains(thisPtr, XDI_FrameRange);
}

/*! \brief Add/update packet counter data for the item
	\param counter The new data to set
*/
void XsDataPacket_setPacketCounter(XsDataPacket* thisPtr, uint16_t counter)
{
	GenericSimple<uint16_t>::set(thisPtr, counter, XDI_PacketCounter);
	MAP.erase(XDI_FrameRange);
}

/*! \brief Return the fine sample time of a packet

	\returns Returns the fine sample time of a packet
*/
uint32_t XsDataPacket_sampleTimeFine(const XsDataPacket* thisPtr)
{
	return GenericSimple<uint32_t>::get(thisPtr, XDI_SampleTimeFine);
}

/*! \brief Check if data item XsDataPacket_contains a sample time fine
	\returns true if this packet XsDataPacket_contains a sample time fine
*/
int XsDataPacket_containsSampleTimeFine(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_SampleTimeFine);
}

/*! \brief Add/update sample tine fine data for the item
	\param counter The new data to set
*/
void XsDataPacket_setSampleTimeFine(XsDataPacket* thisPtr, uint32_t counter)
{
	GenericSimple<uint32_t>::set(thisPtr, counter, XDI_SampleTimeFine);
	auto it = MAP.find(XDI_SampleTime64);
	if (it != MAP.end())
	{
		auto& var = it->second->toDerived<SimpleVariant<uint64_t>>();
		var.m_data = coarseFactor*(var.m_data/coarseFactor) + (counter%coarseFactor);
	}
}

/*! \brief Return the coarse sample time of a packet
*/
uint32_t XsDataPacket_sampleTimeCoarse(const XsDataPacket* thisPtr)
{
	return GenericSimple<uint32_t>::get(thisPtr, XDI_SampleTimeCoarse);
}

/*! \brief Check if data item XsDataPacket_contains a sample time coarse
	\returns true if this packet XsDataPacket_contains a sample time coarse
*/
int XsDataPacket_containsSampleTimeCoarse(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_SampleTimeCoarse);
}

/*! \brief Add/update sample tine coarse data for the item
	\param counter The new data to set
*/
void XsDataPacket_setSampleTimeCoarse(XsDataPacket* thisPtr, uint32_t counter)
{
	GenericSimple<uint32_t>::set(thisPtr, counter, XDI_SampleTimeCoarse);
	auto it = MAP.find(XDI_SampleTime64);
	if (it != MAP.end())
	{
		auto& var = it->second->toDerived<SimpleVariant<uint64_t>>();
		var.m_data = (coarseFactor * counter) + (var.m_data % coarseFactor);
	}
}

/*! \brief Return the full 64-bit sample time of a packet, combined from the fine and coarse sample times or received directly from the device. The 64-bit sample time runs at 10kHz.
*/
uint64_t XsDataPacket_sampleTime64(const XsDataPacket* thisPtr)
{
	auto it = MAP.find(XDI_SampleTime64);
	if (it != MAP.end())
		return it->second->toDerived<SimpleVariant<uint64_t>>().m_data;

	uint64_t rv = 0;

	bool hasSampletimeCoarse = false;
	it = MAP.find(XDI_SampleTimeCoarse);
	if (it != MAP.end())
	{
		hasSampletimeCoarse = true;
		rv += ((uint64_t) it->second->toDerived<SimpleVariant<uint32_t>>().m_data) * coarseFactor;
	}

	it = MAP.find(XDI_SampleTimeFine);
	if (it != MAP.end())
	{
		uint32_t sampletimeFine = it->second->toDerived<SimpleVariant<uint32_t>>().m_data;
		if (hasSampletimeCoarse)
			sampletimeFine = sampletimeFine % coarseFactor;
		rv += sampletimeFine;
	}

	return rv;
}

/*! \brief Check if data item XsDataPacket contains a full 64-bit sample time
	\returns true if this packet XsDataPacket contains both a fine and coarse sample time
*/
int XsDataPacket_containsSampleTime64(const XsDataPacket* thisPtr)
{
	return	genericContains(thisPtr, XDI_SampleTime64) ||
			(genericContains(thisPtr, XDI_SampleTimeCoarse) && genericContains(thisPtr, XDI_SampleTimeFine));
}

/*! \brief Add/update sample tine coarse data for the item
	\param counter The new data to set
*/
void XsDataPacket_setSampleTime64(XsDataPacket* thisPtr, uint64_t counter)
{
	GenericSimple<uint64_t>::set(thisPtr, counter, XDI_SampleTime64);
	GenericSimple<uint32_t>::set(thisPtr, (uint32_t) (counter / coarseFactor), XDI_SampleTimeCoarse);
	GenericSimple<uint32_t>::set(thisPtr, (uint32_t) (counter % coarseFactor), XDI_SampleTimeFine);
}

/*! \brief The utc time component of a data item.
	\param returnVal : The XsTimeInfo to return the requested data in
	\returns An XsTimeInfo containing the utc time value
*/
XsTimeInfo* XsDataPacket_utcTime(const XsDataPacket* thisPtr, XsTimeInfo* returnVal)
{
	return genericGet<XsTimeInfo, XsTimeInfoVariant>(thisPtr, returnVal, XDI_UtcTime);
}

/*! \brief Check if data item contains utc time data
  \returns true if this packet contains utc time data
*/
int XsDataPacket_containsUtcTime(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_UtcTime);
}

/*! \brief Add/update utc time data for the item
	\param data The new data to set
*/
void XsDataPacket_setUtcTime(XsDataPacket* thisPtr, const XsTimeInfo* data)
{
	genericSet<XsTimeInfo, XsTimeInfoVariant>(thisPtr, data, XDI_UtcTime);
}

/*! \brief The free acceleration component of a data item.
	\details Free acceleration is the acceleration with the local gravity vector subtracted.
	\param returnVal : An XsVector to put the requested in

	\returns An XsVector containing the gravity acceleration
*/
XsVector* XsDataPacket_freeAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector3Variant>(thisPtr, returnVal, XDI_FreeAcceleration);
}

/*! \brief Check if data item contains free acceleration
	\details Free acceleration is the acceleration with the local gravity vector subtracted.
	\returns true if this packet contains free acceleration
*/
int XsDataPacket_containsFreeAcceleration(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_FreeAcceleration);
}

/*! \brief Add/update free acceleration data for the item
	\details Free acceleration is the acceleration with the local gravity vector subtracted.
	\param g A 3-component vector containing the new free acceleration
*/
void XsDataPacket_setFreeAcceleration(XsDataPacket* thisPtr, const XsVector* g)
{
	genericSet<XsVector, XsVector3Variant>(thisPtr, g, XDI_FreeAcceleration);
}

/*! \brief Returns the frame range contained in the datapacket

	\param returnVal : The XsRange object that will get the range from the packet

	\returns Returns an XsRange object with the range from the packet
*/
XsRange* XsDataPacket_frameRange(const XsDataPacket* thisPtr, XsRange* returnVal)
{
	return genericGet<XsRange, XsRangeVariant>(thisPtr, returnVal, XDI_FrameRange);
}

/*! \brief Returns whether the datapacket contains a framerange

	\returns Whether the datapacket contains a framerange
*/
int XsDataPacket_containsFrameRange(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_FrameRange);
}

/*! \brief Sets or updates the frame range in the datapacket

	\param r : The XsRange object that should be added to the packet
*/
void XsDataPacket_setFrameRange(XsDataPacket* thisPtr, const XsRange* r)
{
	genericSet<XsRange, XsRangeVariant>(thisPtr, r, XDI_FrameRange);
	MAP.erase(XDI_PacketCounter);
}

/*! \brief Returns the rssi value contained in the datapacket

	\returns Returns the rssi value contained in the datapacket
*/
int XsDataPacket_rssi(const XsDataPacket* thisPtr)
{
	return (int) (int8_t) GenericSimple<uint8_t>::get(thisPtr, XDI_Rssi, (uint8_t) XS_RSSI_UNKNOWN);
}

/*! \brief Returns whether the datapacket contains an rssi value

	\returns Whether the datapacket contains an rssi value
*/
int XsDataPacket_containsRssi(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_Rssi);
}

/*! \brief Sets or updates the rssi value in the datapacket

	\param r : The rssi value that should be added to the packet
*/
void XsDataPacket_setRssi(XsDataPacket* thisPtr, int r)
{
	GenericSimple<uint8_t>::set(thisPtr, (uint8_t)(int8_t)r, XDI_Rssi);
}

/*! \brief Returns a struct with RawGnssPvtData
	\param returnVal The object to store the requested data in
	\return a struct with RawGnssPvtData
*/
XsRawGnssPvtData* XsDataPacket_rawGnssPvtData(const XsDataPacket* thisPtr, XsRawGnssPvtData* returnVal)
{
	return genericGet<XsRawGnssPvtData, XsRawGnssPvtDataVariant>(thisPtr, returnVal, XDI_GnssPvtData);
}

/*! \brief Returns 1 if data item contains RawGnssPvtData, 0 otherwise
*/
int XsDataPacket_containsRawGnssPvtData(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_GnssPvtData);
}

/*! \brief Sets or updates the RawGnssPvtData value in the datapacket
	\param r The new value that should be added to the packet
*/
void XsDataPacket_setRawGnssPvtData(XsDataPacket* thisPtr, const XsRawGnssPvtData* r)
{
	genericSet<XsRawGnssPvtData, XsRawGnssPvtDataVariant>(thisPtr, r, XDI_GnssPvtData);
}


/*! \brief Returns the age of the GNSS data (in samples)
*/
uint8_t XsDataPacket_gnssAge(const XsDataPacket* thisPtr)
{
	return GenericSimple<uint8_t>::get(thisPtr, XDI_GnssAge, 255);
}

/*! \brief Returns 1 if data item contains GnssAge, 0 otherwise
*/
int XsDataPacket_containsGnssAge(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_GnssAge);
}

/*! \brief Sets or updates the GnssAge value in the datapacket
	\param age The new value that should be added to the packet
*/
void XsDataPacket_setGnssAge(XsDataPacket* thisPtr, uint8_t age)
{
	GenericSimple<uint8_t>::set(thisPtr, age, XDI_GnssAge);
}

/*! \brief Returns a struct with RawGnssSatInfo
	\param returnVal The object to store the requested data in
	\return a struct with RawGnssSatInfo
*/
XsRawGnssSatInfo* XsDataPacket_rawGnssSatInfo(const XsDataPacket* thisPtr, XsRawGnssSatInfo* returnVal)
{
	return genericGet<XsRawGnssSatInfo, XsRawGnssSatInfoVariant>(thisPtr, returnVal, XDI_GnssSatInfo);
}

/*! \brief Returns 1 if data item contains RawGnssPvtData, 0 otherwise
*/
int XsDataPacket_containsRawGnssSatInfo(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_GnssSatInfo);
}

/*! \brief Sets or updates the RawGnssSatInfo value in the datapacket
	\param data The new value that should be added to the packet
*/
void XsDataPacket_setRawGnssSatInfo(XsDataPacket* thisPtr, const XsRawGnssSatInfo* data)
{
	genericSet<XsRawGnssSatInfo, XsRawGnssSatInfoVariant>(thisPtr, data, XDI_GnssSatInfo);
}

/*! \brief Merge the data items from \a other into the packet

	\details The data items contained in XsDataPacket \a other will be added to
			this packet. Items that are already present will be
			overwritten depending on the value of \a overwrite.

	\param other The XsDataPacket to read the data items from
	\param overwrite When set to true, existing items will be overwritten, otherwise they will not be modified

	\returns Returns the updated data packet
*/
XsDataPacket* XsDataPacket_merge(XsDataPacket* thisPtr, const XsDataPacket* other, int overwrite)
{
	detach(thisPtr);
	bool over = !!overwrite;
	thisPtr->d->merge(*other->d, over);

	// do additional handling for 'unique' items
	auto keepOne = [&](XsDataIdentifier id1, XsDataIdentifier id2)
	{
		if (genericContains(thisPtr, id1) &&
			genericContains(thisPtr, id2))
		{
			bool gc = genericContains(other, id1);
			if ((gc && !over) || (!gc && over))	// logical xor does not exist in C++, write it explicitly
				MAP.erase(id1);
			else
				MAP.erase(id2);
		}
	};

	keepOne(XDI_Quaternion, XDI_RotationMatrix);
	keepOne(XDI_Quaternion, XDI_EulerAngles);
	keepOne(XDI_EulerAngles, XDI_RotationMatrix);
	keepOne(XDI_PacketCounter, XDI_FrameRange);
	keepOne(XDI_SampleTime64, XDI_SampleTimeCoarse);
	keepOne(XDI_SampleTime64, XDI_SampleTimeFine);

	if (over)
	{
		thisPtr->m_deviceId = other->m_deviceId;
		thisPtr->m_toa = other->m_toa;
		thisPtr->m_packetId = other->m_packetId;
		thisPtr->m_etos = other->m_etos;
	}

	return thisPtr;
}

/*!	\brief Overwrite the contents of the XsDataPacket with the contents of the supplied XsMessage
	\param msg The XsMessage to read from
	\note The packet is cleared before inserting new items
*/
void XsDataPacket_setMessage(XsDataPacket* thisPtr, const XsMessage* msg)
{
	XsDataPacket_clear(thisPtr, XDI_None);

	XsSize offset = 0;
	XsSize sz = msg->getDataSize();

	while (offset+3 <= sz)	// minimum size of an item is 2(ID) + 1(size) + 0(minimum size)
	{
		XsDataIdentifier id = static_cast<XsDataIdentifier>(XsMessage_getDataShort(msg, offset));
		XsSize itemSize = XsMessage_getDataByte(msg, offset+2);
		if (offset + itemSize + 3 > sz)
			break;	// the item is corrupt

		Variant* var = createVariant(id);
		if (var)
		{
			itemSize = var->readFromMessage(*msg, offset+3, itemSize);
			MAP.insert(id, var);
		}
		offset += 3 + itemSize;	// never use var->sizeInMsg() here, since it _may_ differ
	}
	if (offset < sz)
	{
		// if we get here then we detected some kind of corruption
		// so we should not trust anything we read
		XsDataPacket_clear(thisPtr, XDI_None);
		return;
	}
}

/*! \brief Write the contents of the XsDataPacket to an XsMessage in MtData2 format
	\details The function will clear the message and add all items in the XsDataPacket as a standard MtData2 message.
	\param msg The message to write to
*/
void XsDataPacket_toMessage(const XsDataPacket* thisPtr, XsMessage* msg)
{
	msg->resizeData(0);	// clear the data part while leaving header intact
	msg->setMessageId(XMID_MtData2);

	XsSize offset = 0;
	msg->resizeData(2048);	// prevent constant message resizing by pre-allocating a large message and later reducing its size
	for (auto const& i : MAP)
	{
		XsSize sz = i.second->sizeInMsg();
		if (sz < 255)
		{
			msg->setDataShort((uint16_t) i.second->dataId(), offset);
			msg->setDataByte((uint8_t)sz, offset+2);
			i.second->writeToMessage(*msg, offset+3);
			offset += 3+sz;
		}
		else
		{
			XsSize sz2 = sz;
			XsSize offset2 = offset;
			while (sz2 >= 255)
			{
				msg->setDataShort((uint16_t) i.second->dataId(), offset2);
				msg->setDataByte((uint8_t)255, offset2+2);
				offset2 += 258;
				sz2 -= 255;
			}
			msg->setDataShort((uint16_t) i.second->dataId(), offset2);
			msg->setDataByte((uint8_t)sz2, offset2+2);	// note that this size may be 0
			i.second->writeToMessage(*msg, offset+3);	// individual write functions should takke extended size into account
			offset = offset2+3+sz2;
		}
	}
	msg->resizeData(offset);
}

/*! \brief Returns the Full Snapshot part of the XsDataPacket
	\details Full Snapshot is an internal format used by Xsens devices for high accuracy data transfer.
	In most cases XDA processing will remove this item from the XsDataPacket and replace it with items that
	are more directly usable.
	\param returnVal The object to store the requested data in. This must be a properly constructed object.
	\returns The supplied \a returnVal, filled with the requested data or cleared if it was not available
*/
XsSnapshot* XsDataPacket_fullSnapshot(const XsDataPacket* thisPtr, XsSnapshot* returnVal)
{
	return genericGet<XsSnapshot, XsFullSnapshotVariant>(thisPtr, returnVal, XDI_FullSnapshot);
}

/*! \brief Returns true if the XsDataPacket contains Full Snapshot data
	\returns true if the XsDataPacket contains Full Snapshot data
*/
int XsDataPacket_containsFullSnapshot(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_FullSnapshot);
}

/*! \brief Sets the Awinda Snapshot part of the XsDataPacket
	\param data The new data to set
	\param retransmission When non-zero, the item is marked as a retransmitted packet
*/
void XsDataPacket_setFullSnapshot(XsDataPacket* thisPtr, XsSnapshot const * data, int retransmission)
{
	genericSet<XsSnapshot, XsFullSnapshotVariant>(thisPtr, data, XDI_FullSnapshot | (retransmission ? XDI_RetransmissionFlag : XDI_None));
}

/*! \brief Returns the Awinda Snapshot part of the XsDataPacket
	\details Awinda Snapshot is an internal format used by Xsens devices for high accuracy data trasnfer.
	In most cases XDA processing will remove this item from the XsDataPacket and replace it with items that
	are more directly usable.
	\param returnVal The object to store the requested data in. This must be a properly constructed object.
	\returns The supplied \a returnVal, filled with the requested data or cleared if it was not available
*/
XsSnapshot* XsDataPacket_awindaSnapshot(const XsDataPacket* thisPtr, XsSnapshot* returnVal)
{
	return genericGet<XsSnapshot, XsAwindaSnapshotVariant>(thisPtr, returnVal, XDI_AwindaSnapshot);
}

/*! \brief Returns true if the XsDataPacket contains Awinda Snapshot data
	\returns true if the XsDataPacket contains Awinda Snapshot data
*/
int XsDataPacket_containsAwindaSnapshot(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_AwindaSnapshot);
}

/*! \brief Sets the Awinda Snapshot part of the XsDataPacket
	\param data The new data to set
	\param retransmission When non-zero, the item is marked as a retransmitted packet
*/
void XsDataPacket_setAwindaSnapshot(XsDataPacket* thisPtr, XsSnapshot const * data, int retransmission)
{
	genericSet<XsSnapshot, XsAwindaSnapshotVariant>(thisPtr, data, XDI_AwindaSnapshot | (retransmission ? XDI_RetransmissionFlag : XDI_None));
}

/*! \brief Returns true if the contained Awinda Snapshot is marked as a retransmission
	\returns true if the XsDataPacket contains Awinda Snapshot data and it is marked as a retransmission
*/
int XsDataPacket_isAwindaSnapshotARetransmission(const XsDataPacket* thisPtr)
{
	auto it = MAP.find(XDI_AwindaSnapshot);
	if (it == MAP.end())
		return false;
	return (it->second->dataId() & XDI_RetransmissionMask) == XDI_RetransmissionFlag;
}

/*! \brief Returns the Glove Snapshot part of the XsDataPacket
\details Glove Snapshot is an internal format used by Xsens devices for high accuracy data transfer.
In most cases XDA processing will remove this item from the XsDataPacket and replace it with items that
are more directly usable.
\param returnVal The object to store the requested data in. This must be a properly constructed object.
\returns The supplied \a returnVal, filled with the requested data or cleared if it was not available
*/
XsGloveSnapshot* XsDataPacket_gloveSnapshot(const XsDataPacket* thisPtr, XsGloveSnapshot* returnVal)
{
	return genericGet<XsGloveSnapshot, XsGloveSnapshotVariant>(thisPtr, returnVal, XDI_GloveSnapshot);
}

/*! \brief Returns true if the XsDataPacket contains Glove Snapshot data
\returns true if the XsDataPacket contains Glove Snapshot data
*/
int XsDataPacket_containsGloveSnapshot(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_GloveSnapshot);
}

/*! \brief Sets the Glove Snapshot part of the XsDataPacket
\param data The new data to set
\param retransmission When non-zero, the item is marked as a retransmitted packet
*/
void XsDataPacket_setGloveSnapshot(XsDataPacket* thisPtr, XsGloveSnapshot const * data, int retransmission)
{
	genericSet<XsGloveSnapshot, XsGloveSnapshotVariant>(thisPtr, data, XDI_GloveSnapshot | (retransmission ? XDI_RetransmissionFlag : XDI_None));
}

/*! \brief Converts input vector \a input with data identifier \a id to output XsVector \a returnVal */
static void convertRawVector(XsUShortVector const& input, XsDataIdentifier id, XsVector& returnVal)
{
	XsReal (*caster)(uint16_t) = unsigned_cast;
	returnVal.setSize(3);
	if ((id & XDI_DataFormatMask) == XDI_RawSigned)
		caster = signed_cast;
	for (XsSize i = 0; i < 3; i++)
		returnVal[i] = caster(input[i]);
}

/*! \returns the data component specified by \a id and \a field, converted to floating point values
	\param[in] id The data identifier of the data component
	\param[in] field Pointer to a member in XsScrData, pointing to the data to retrieve
	\param[out] returnVal the returned floating point values
*/
static XsVector* convertedVector(const XsDataPacket* thisPtr, XsVector* returnVal, XsDataIdentifier id, XsUShortVector XsScrData::* field)
{
	assert(returnVal);

	XsUShortVector tmp;

	auto it = MAP.find(XDI_RawAccGyrMagTemp);
	if (it != MAP.end())
		tmp = it->second->toDerived<XsScrDataVariant>().m_data.*field;
	else
	{
		it = MAP.find(id);
		if (it != MAP.end())
			tmp = it->second->toDerived<XsUShortVectorVariant>().m_data;
	}

	convertRawVector(tmp, it->second->dataId(), *returnVal);
	return returnVal;
}

/*!	\brief The raw accelerometer component of a data item, converted to floating point values.

	\param returnVal : An XsVector to put the requested data in

	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_rawAccelerationConverted(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return convertedVector(thisPtr, returnVal, XDI_RawAcc, &XsScrData::m_acc);
}

/*!	\brief The raw gyroscope component of a data item, converted to floating point values.

	\param returnVal : An XsVector to put the requested data in

	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_rawGyroscopeDataConverted(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return convertedVector(thisPtr, returnVal, XDI_RawGyr, &XsScrData::m_gyr);
}

/*!	\brief The raw magnetometer component of a data item, converted to floating point values.

	\param returnVal : An XsVector to put the requested data in

	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_rawMagneticFieldConverted(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return convertedVector(thisPtr, returnVal, XDI_RawMag, &XsScrData::m_mag);
}

/*!	\brief The raw gyroscope temperature component of a data item, converted to floating point values.

	\param returnVal : An XsVector to put the requested data in

	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_rawGyroscopeTemperatureDataConverted(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	assert(returnVal);
	auto it = MAP.find(XDI_RawGyroTemp);
	XsUShortVector tmp;
	if (it != MAP.end())
		tmp = it->second->toDerived<XsUShortVectorVariant>().m_data;
	convertRawVector(tmp, it->second->dataId(), *returnVal);
	return returnVal;
}

/*! \brief Returns the raw blob part of the XsDataPacket
	\param returnVal The object to store the requested data in. This must be a properly constructed object.
	\returns The supplied \a returnVal, filled with the requested data or cleared if it was not available
*/
XsByteArray* XsDataPacket_rawBlob(const XsDataPacket* thisPtr, XsByteArray* returnVal)
{
	return genericGet<XsByteArray, XsByteArrayVariant>(thisPtr, returnVal, XDI_RawBlob);
}

/*! \brief Returns true if the XsDataPacket contains raw blob data
	\returns true if the XsDataPacket contains raw blob data
*/
int XsDataPacket_containsRawBlob(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_RawBlob);
}

/*! \brief Sets the raw blob part of the XsDataPacket
	\param data The new data to set
*/
void XsDataPacket_setRawBlob(XsDataPacket* thisPtr, const XsByteArray *data)
{
	genericSet<XsByteArray, XsByteArrayVariant>(thisPtr, data, XDI_RawBlob);
}

/*! \brief Returns AccelerationHR
	\param returnVal : An XsVector to put the requested in
	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_accelerationHR(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector3Variant>(thisPtr, returnVal, XDI_AccelerationHR);
}

/*! \brief Check if data item contains AccelerationHR
	\returns true if this packet contains AccelerationHR
*/
int XsDataPacket_containsAccelerationHR(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_AccelerationHR);
}

/*! \brief Add/update the AccelerationHR for the item
	\param vec A 3-component vector containing AccelerationHR
*/
void XsDataPacket_setAccelerationHR(XsDataPacket* thisPtr, const XsVector* vec)
{
	genericSet<XsVector, XsVector3Variant>(thisPtr, vec, XDI_AccelerationHR | XDI_SubFormatDouble);
}

/*! \brief Returns RateOfTurnHR
	\param returnVal : An XsVector to put the requested in
	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_rateOfTurnHR(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	return genericGet<XsVector, XsVector3Variant>(thisPtr, returnVal, XDI_RateOfTurnHR);
}

/*! \brief Check if data item contains RateOfTurnHR
	\returns true if this packet contains RateOfTurnHR
*/
int XsDataPacket_containsRateOfTurnHR(const XsDataPacket* thisPtr)
{
	return genericContains(thisPtr, XDI_RateOfTurnHR);
}

/*! \brief Add/update the RateOfTurnHR for the item
	\param vec A 3-component vector containing RateOfTurnHR
*/
void XsDataPacket_setRateOfTurnHR(XsDataPacket* thisPtr, const XsVector* vec)
{
	genericSet<XsVector, XsVector3Variant>(thisPtr, vec, XDI_RateOfTurnHR | XDI_SubFormatDouble);
}

/*! \brief Returns the number of private data items for all XsDataPacket combined currently in memory
	\details This is an internal value and should not be used for any other purpose.
	\return The current number of items.
*/
int XsDataPacket_privateCount()
{
	return DataPacketPrivate::creationDiff();
}

}	// extern "C"

/*! @} */
