/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsdatapacket.h"
#include "xsmatrix3x3.h"
#include "xsrssi.h"
#include "xsmath.h"
#include <string.h>

/*! \class XsDataPacket
	\brief Contains an interpreted data message. The class provides easy access to the contained
	data through its many functions.
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Check if the packet is internally consistent
	\details This function will throw an exception if there are inconsistencies in the size of the
	message buffers and the size as reported by the header.
*/
void validatePacket(XsDataPacket* thisPtr)
{
	(void)thisPtr;
	assert(thisPtr->m_msg.m_message.m_size >= thisPtr->m_legacyMsg.m_message.m_size);
	assert(XsMessage_getTotalMessageSize(&thisPtr->m_msg) >= XsMessage_getTotalMessageSize(&thisPtr->m_legacyMsg));
}

/*! \relates XsDataPacket
	\brief Inits a data packet, the packet will be empty after construction
*/
void XsDataPacket_construct(XsDataPacket* thisPtr)
{
	XsMessage_construct(&thisPtr->m_msg);
	XsMessage_setMessageId(&thisPtr->m_msg, XMID_InvalidMessage);

	XsMessage_construct(&thisPtr->m_legacyMsg);
	XsMessage_setMessageId(&thisPtr->m_legacyMsg, XMID_InvalidMessage);
	thisPtr->m_lastFoundId = XDI_None;
	thisPtr->m_lastFoundOffset = -1;
	thisPtr->m_itemCount = 0;
}

/*! \relates XsDataPacket
	\brief Clears and frees data in an XsDataPacket
*/
void XsDataPacket_destruct(XsDataPacket* thisPtr)
{
	XsMessage_destruct(&thisPtr->m_msg);
	XsMessage_destruct(&thisPtr->m_legacyMsg);
}

/*! \relates XsDataPacket
	\brief Copy the XsDataPacket to \a copy
*/
void XsDataPacket_copy(XsDataPacket* copy, XsDataPacket const* src)
{
	XsMessage_copy(&copy->m_msg, &src->m_msg);
	XsMessage_copy(&copy->m_legacyMsg, &src->m_legacyMsg);
	copy->m_deviceId = src->m_deviceId;
	copy->m_lastFoundId = src->m_lastFoundId;
	copy->m_lastFoundOffset = src->m_lastFoundOffset;
	copy->m_itemCount = src->m_itemCount;
	copy->m_originalMessageLength = src->m_originalMessageLength;
	copy->m_toa = src->m_toa;
	copy->m_packetId = src->m_packetId;
}

/*! \relates XsDataPacket
	\brief Swaps the XsDataPackets in \a a and \a b
*/
void XsDataPacket_swap(XsDataPacket* a, XsDataPacket* b)
{
	XsDeviceId tdid;
	XsDataIdentifier tdi;
	uint16_t t16;
	XsTimeStamp tts;

	XsMessage_swap(&a->m_msg, &b->m_msg);
	XsMessage_swap(&a->m_legacyMsg, &b->m_legacyMsg);

	tdid = b->m_deviceId;
	b->m_deviceId = a->m_deviceId;
	a->m_deviceId = tdid;

	tdi = b->m_lastFoundId;
	b->m_lastFoundId = a->m_lastFoundId;
	a->m_lastFoundId = tdi;

	t16 = b->m_lastFoundOffset;
	b->m_lastFoundOffset = a->m_lastFoundOffset;
	a->m_lastFoundOffset = t16;

	t16 = b->m_itemCount;
	b->m_itemCount = a->m_itemCount;
	a->m_itemCount = t16;

	t16 = b->m_originalMessageLength;
	b->m_originalMessageLength = a->m_originalMessageLength;
	a->m_originalMessageLength = t16;

	tts = b->m_toa;
	b->m_toa = a->m_toa;
	a->m_toa = tts;

	tts = b->m_packetId;
	b->m_packetId = a->m_packetId;
	a->m_packetId = tts;
}

/*! \relates XsDataPacket
	\brief Returns whether the XsDataPacket is empty
*/
int XsDataPacket_empty(const XsDataPacket* thisPtr)
{
	return 0 == thisPtr->m_itemCount;
}

/*!	\relates XsDataPacket
	\brief Return the item offset of the supplied data identifier \a id using matching criteria specified by \a mask
	\details This function will ignore the bits not in the mask when searching for \a id
	\param id The data identifier to return the offset off
	\param mask The bits to check for
	\returns The offset in the message where the data of this identifier can be found or -1 if it could not be found
	\sa XsDataPacket_itemOffsetExact \sa XsDataPacket_itemOffsetLoose
*/
int XsDataPacket_itemOffsetMasked(const struct XsDataPacket* thisPtr, XsDataIdentifier id, XsDataIdentifier mask)
{
	XsSize offset = 0;
	XsSize dataSize;

	id = (id & mask);

	if (id == (thisPtr->m_lastFoundId & mask))
		return thisPtr->m_lastFoundOffset;

	((XsDataPacket*)thisPtr)->m_lastFoundOffset = -1;
	((XsDataPacket*)thisPtr)->m_lastFoundId = XDI_None;

	dataSize = XsMessage_dataSize(&thisPtr->m_msg);
	while (offset < dataSize)
	{
		XsDataIdentifier cid = (XsDataIdentifier) XsMessage_getDataShort(&thisPtr->m_msg, offset);
		if ((cid & mask) == id)
		{
			((XsDataPacket*)thisPtr)->m_lastFoundId = cid;
			((XsDataPacket*)thisPtr)->m_lastFoundOffset = (int) (offset + 3);
			break;
		}
		offset += 3 + XsMessage_getDataByte(&thisPtr->m_msg, offset+2);
	}
	return thisPtr->m_lastFoundOffset;
}

/*!	\relates XsDataPacket
	\brief Return the item offset of the supplied data identifier \a id using strict matching criteria
	\details This function will search for an exact match of \a id, including the subformat.
	Equivalent to XsDataPacket_itemOffsetMasked(thisPtr, id, XDI_FullMask);
	\param id The data identifier to return the offset off
	\returns The offset in the message where the data of this identifier can be found or -1 if it could not be found
	\sa XsDataPacket_itemOffsetLoose \sa XsDataPacket_itemOffsetMasked
*/
int XsDataPacket_itemOffsetExact(const XsDataPacket* thisPtr, XsDataIdentifier id)
{
	return XsDataPacket_itemOffsetMasked(thisPtr, id, XDI_FullMask);
}

/*! \relates XsDataPacket
	\brief Return the item offset of the supplied data identifier \a id using loose matching criteria
	\details This function will ignore the subformat of the data when searching for \a id.
	Equivalent to XsDataPacket_itemOffsetMasked(thisPtr, id, ~XDI_SubFormatMask);
	\param id The data identifier to return the offset off
	\returns The offset in the message where the data of this identifier can be found or -1 if it could not be found
	\sa XsDataPacket_itemOffsetExact
*/
int XsDataPacket_itemOffsetLoose(const XsDataPacket* thisPtr, XsDataIdentifier id)
{
	return XsDataPacket_itemOffsetMasked(thisPtr, id, XDI_FullTypeMask);	//lint !e64
}

/*!	\relates XsDataPacket
	\brief Sets a message in a datapacket

	\param msg The XsMessage to set
*/
void XsDataPacket_setMessage(XsDataPacket* thisPtr, const XsMessage* msg)
{
	uint16_t offset = 0;
	thisPtr->m_itemCount = 0;
	XsMessage_copy(&thisPtr->m_msg, msg);
	thisPtr->m_originalMessageLength = (uint16_t) XsMessage_dataSize(&thisPtr->m_msg);
	while (offset < thisPtr->m_originalMessageLength)
	{
		//uint16_t id = XsMessage_getDataShort(&thisPtr->m_msg, offset);
		uint8_t size = XsMessage_getDataByte(&thisPtr->m_msg, offset+2);
		offset += 3 + size;
		++thisPtr->m_itemCount;
	}
}

/*! \brief Returns a copy of the original message of the data packet
	\details This returns the original message that was last set with setMessage,
	or in the constructor.
	When the packet was constructed from a legacy message, the legacy message will be returned.
	\param returnVal The %XsMessage that the message of the packet is copied to
	\returns returnVal
*/
XsMessage* XsDataPacket_originalMessage(const XsDataPacket* thisPtr, XsMessage* returnVal)
{
	const XsMessageHeader* hdr;

	assert(returnVal);
	if (!XsMessage_empty(&thisPtr->m_legacyMsg))
	{
		XsMessage_copy(returnVal, &thisPtr->m_legacyMsg);
		return returnVal;
	}

	if (thisPtr->m_originalMessageLength == (uint16_t) XsMessage_dataSize(&thisPtr->m_msg))
	{
		XsMessage_copy(returnVal, &thisPtr->m_msg);
		return returnVal;
	}

	returnVal->m_autoUpdateChecksum = 1;
	XsMessage_assign(returnVal, thisPtr->m_originalMessageLength);
	XsMessage_setDataBuffer(returnVal, XsMessage_getDataBuffer(&thisPtr->m_msg, 0), thisPtr->m_originalMessageLength, 0);

	hdr = XsMessage_getConstHeader(&thisPtr->m_msg);
	XsMessage_setBusId(returnVal, hdr->m_busId);
	XsMessage_setMessageId(returnVal, (XsXbusMessageId) hdr->m_messageId);
	return returnVal;
}

/*!	\relates XsDataPacket
	\brief Returns the dataformat of a specific data identifier in the packet

	\param id : The XsDataIdentifier to query
	\returns Returns XDI_None if the packet does not contain the dataidentifier, the data
	format otherwise

	\sa XsDataIdentifier
*/
XsDataIdentifier XsDataPacket_dataFormat(const XsDataPacket* thisPtr, XsDataIdentifier id)
{
	if (XsDataPacket_itemOffsetLoose(thisPtr, id) == -1)
		return XDI_None;
	return thisPtr->m_lastFoundId & XDI_SubFormatMask;
}

/*!	\relates XsDataPacket
	\brief Returns the byte size of \a id if the format is a floating point format

	\param id : The XsDataIdentifier to query
	\returns Returns the byte size of XsDataIdentifier \a id
*/
uint8_t XsDataPacket_getFPValueSize(XsDataIdentifier id)
{
	switch (id & XDI_SubFormatMask)
	{
		case XDI_SubFormatFloat:
			return 4;

		case XDI_SubFormatDouble:
			return 8;

		case XDI_SubFormatFp1632:
			return 6;

		case XDI_SubFormatFp1220:
			return 4;

		default:
			return 0;
	}
}

/*!	\relates XsDataPacket
	\brief The raw accelerometer component of a data item.

	\param returnVal : An XsUShortVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector* XsDataPacket_rawAcceleration(const XsDataPacket* thisPtr, XsUShortVector* returnVal)
{
	XsSize i;
	assert(returnVal);
	if (XsDataPacket_containsRawAcceleration(thisPtr))
		for (i=0 ; i<3 ; ++i)
			returnVal->m_data[i] = XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + (2*i));

	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains Raw Accelerometer data
	\returns true if this packet contains raw acceleration data
*/
int XsDataPacket_containsRawAcceleration(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update raw accelerometer data for the item

	\param vec : The data to update the XsDataPacket with

	\details This will add the raw acceleration from \a vec to the data packet. If
			 the packet already contains raw acceleration, it will be replaced.
*/
void XsDataPacket_setRawAcceleration(XsDataPacket* thisPtr, const XsUShortVector* vec)
{
	XsSize i;
	if (!XsDataPacket_containsRawAcceleration(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+23);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RawAccGyrMagTemp, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 20, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	for (i=0 ; i<3 ; ++i)
		XsMessage_setDataShort(&thisPtr->m_msg, vec->m_data[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + (2*i));
}

/*! \relates XsDataPacket
	\brief The raw gyroscope component of a data item.

	\param returnVal : An XsUShortVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector* XsDataPacket_rawGyroscopeData(const XsDataPacket* thisPtr, XsUShortVector* returnVal)
{
	XsSize i;
	if (XsDataPacket_containsRawGyroscopeData(thisPtr))
		for (i=0 ; i<3 ; ++i)
			returnVal->m_data[i] = XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + 6 + (2*i));

	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains raw gyroscope data
	\returns true if this packet contains raw gyroscope data
*/
int XsDataPacket_containsRawGyroscopeData(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update raw gyroscope data for the item
*/
void XsDataPacket_setRawGyroscopeData(XsDataPacket* thisPtr, const XsUShortVector* vec)
{
	XsSize i;
	if (!XsDataPacket_containsRawGyroscopeData(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+23);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RawAccGyrMagTemp, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 20, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	for (i=0 ; i<3 ; ++i)
		XsMessage_setDataShort(&thisPtr->m_msg, vec->m_data[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + 6 + (2*i));
}

/*! \relates XsDataPacket
	\brief The raw gyroscope temperature component of a data item.

	\param returnVal : An XsUShortVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector* XsDataPacket_rawGyroscopeTemperatureData(const XsDataPacket* thisPtr, XsUShortVector* returnVal)
{
	XsSize i;
	assert(returnVal);
	if (XsDataPacket_containsRawGyroscopeTemperatureData(thisPtr))
		for (i=0 ; i<3 ; ++i)
			returnVal->m_data[i] = XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_RawGyroTemp) + (2*i));

	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains raw gyroscope temperature data
  \returns true if this packet contains raw gyroscope temperature data
*/
int XsDataPacket_containsRawGyroscopeTemperatureData(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_RawGyroTemp) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update raw gyroscope temperature data for the item
*/
void XsDataPacket_setRawGyroscopeTemperatureData(XsDataPacket* thisPtr, const XsUShortVector* vec)
{
	XsSize i;
	if (!XsDataPacket_containsRawGyroscopeTemperatureData(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+6);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RawGyroTemp, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 6, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	for (i=0 ; i<3 ; ++i)
		XsMessage_setDataShort(&thisPtr->m_msg, vec->m_data[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_RawGyroTemp) + (2*i));
}

/*! \relates XsDataPacket
	\brief The raw magnetometer component of a data item.

	\param returnVal : An XsUShortVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsUShortVector* XsDataPacket_rawMagneticField(const XsDataPacket* thisPtr, XsUShortVector* returnVal)
{
	XsSize i;
	assert(returnVal);
	if (XsDataPacket_containsRawMagneticField(thisPtr))
		for (i=0 ; i<3 ; ++i)
			returnVal->m_data[i] = XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + 12 + (2*i));

	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains raw magnetometer data
  \returns true if this packet contains raw magnetometer data
*/
int XsDataPacket_containsRawMagneticField(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update raw magnetometer data for the item
*/
void XsDataPacket_setRawMagneticField(XsDataPacket* thisPtr, const XsUShortVector* vec)
{
	XsSize i;
	if (!XsDataPacket_containsRawMagneticField(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+23);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RawAccGyrMagTemp, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 20, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	for (i=0 ; i<3 ; ++i)
		XsMessage_setDataShort(&thisPtr->m_msg, vec->m_data[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + 12 + (2*i));
}

/*! \relates XsDataPacket
	\brief The raw temperature component of a data item.

	\returns An uint16_t containing the raw temperature value
*/
uint16_t XsDataPacket_rawTemperature(const XsDataPacket* thisPtr)
{
	if (XsDataPacket_containsRawTemperature(thisPtr))
		return XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp)+18);
	else
		return 0;
}

/*! \relates XsDataPacket
	\brief Check if data item contains raw temperature data
  \returns true if this packet contains raw temperature data
*/
int XsDataPacket_containsRawTemperature(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update raw temperature data for the item
*/
void XsDataPacket_setRawTemperature(XsDataPacket* thisPtr, uint16_t temp)
{
	if (!XsDataPacket_containsRawTemperature(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+23);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RawAccGyrMagTemp, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 20, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataShort(&thisPtr->m_msg, temp, XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp)+18);
}

/*! \relates XsDataPacket
	\brief Return the raw data component of a data item.
*/
XsScrData* XsDataPacket_rawData(const XsDataPacket* thisPtr, XsScrData* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsRawData(thisPtr))
	{
		XsSize i;
		const uint8_t* tmp = XsMessage_getDataBuffer(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp));
		const uint16_t* sh = (const uint16_t*) tmp;
		//uint16_t* bare = (uint16_t*) &returnVal;

		for (i = 0; i < 3; ++i, ++sh)
			returnVal->m_acc.m_data[i] = swapEndian16(*sh);
		for (i = 0; i < 3; ++i, ++sh)
			returnVal->m_gyr.m_data[i] = swapEndian16(*sh);
		for (i = 0; i < 3; ++i, ++sh)
			returnVal->m_mag.m_data[i] = swapEndian16(*sh);
		returnVal->m_temp[0] = swapEndian16(*sh);

		if (XsDataPacket_containsRawGyroscopeTemperatureData(thisPtr))
		{
			const uint16_t* gyrTemp = (uint16_t*) XsMessage_getDataBuffer(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_RawGyroTemp));
			for (i = 0; i < 3; i++)
				returnVal->m_temp[i + 1] = swapEndian16(gyrTemp[i]);
		} else
		{
			for (i = 0; i < 3; i++)
				returnVal->m_temp[i + 1] = returnVal->m_temp[0];
		}

	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains raw data
  \returns true if this packet contains raw data
*/
int XsDataPacket_containsRawData(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update raw data for the item
*/
void XsDataPacket_setRawData(XsDataPacket* thisPtr, const XsScrData* data)
{
	XsSize i;
	if (!XsDataPacket_containsRawData(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+23);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RawAccGyrMagTemp, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 20, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	for (i = 0; i < 3; ++i)
		XsMessage_setDataShort(&thisPtr->m_msg, data->m_acc.m_data[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + (2*i));
	for (i = 0; i < 3; ++i)
		XsMessage_setDataShort(&thisPtr->m_msg, data->m_gyr.m_data[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + (2*i+6));
	for (i = 0; i < 3; ++i)
		XsMessage_setDataShort(&thisPtr->m_msg, data->m_mag.m_data[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + (2*i+12));
	XsMessage_setDataShort(&thisPtr->m_msg, data->m_temp[0], XsDataPacket_itemOffsetExact(thisPtr, XDI_RawAccGyrMagTemp) + 18);
}

/*! \relates XsDataPacket
	\brief The calibrated accelerometer component of a data item.

	\param returnVal : The XsVector that the calibrated acceleration will be assigned to

	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_calibratedAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsCalibratedAcceleration(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_Acceleration);
		XsVector_assign(returnVal, 3, 0);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_data[0], offset, 3);
		return returnVal;
	}
	XsVector_assign(returnVal, 0, 0);
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains calibrated accelerometer data
  \returns true if this packet contains calibrated accelerometer data
*/
int XsDataPacket_containsCalibratedAcceleration(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_Acceleration) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update calibrated accelerometer data for the item
*/
void XsDataPacket_setCalibratedAcceleration(XsDataPacket* thisPtr, const XsVector* vec)
{
	const int numValues = 3;

	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_Acceleration | XDI_SubFormatDouble) == -1)
	{
		// add
		//m_msg.m_autoUpdateChecksum = false;

		XsSize dsz = numValues*XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+dsz);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_Acceleration | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) dsz, sz+2);
		++thisPtr->m_itemCount;
	}

	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &vec->m_data[0], XsDataPacket_itemOffsetExact(thisPtr, XDI_Acceleration | XDI_SubFormatDouble), numValues);
}

/*! \relates XsDataPacket
	\brief The calibrated gyroscope component of a data item.

	\param returnVal : An XsVector to put the requested data in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_calibratedGyroscopeData(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsCalibratedGyroscopeData(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_RateOfTurn);
		XsVector_assign(returnVal, 3, 0);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_data[0], offset, 3);
		return returnVal;
	}
	XsVector_assign(returnVal, 0, 0);
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains calibrated gyroscope data
  \returns true if this packet contains calibrated gyroscope data
*/
int XsDataPacket_containsCalibratedGyroscopeData(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_RateOfTurn) != -1);
}

/*! \brief Add/update calibrated gyroscope data for the item
*/
void XsDataPacket_setCalibratedGyroscopeData(XsDataPacket* thisPtr, const XsVector* vec)
{
	const int numValues = 3;

	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_RateOfTurn | XDI_SubFormatDouble) == -1)
	{
		// add
		//m_msg.m_autoUpdateChecksum = false;

		XsSize dsz = numValues*XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+dsz);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RateOfTurn | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) dsz, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &vec->m_data[0], XsDataPacket_itemOffsetExact(thisPtr, XDI_RateOfTurn | XDI_SubFormatDouble), numValues);
}

/*! \relates XsDataPacket
	\brief The calibrated magnetometer component of a data item.

	\param returnVal : An XsVector to put the requested in

	\returns A XsUShortVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_calibratedMagneticField(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsCalibratedMagneticField(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_MagneticField);
		XsVector_assign(returnVal, 3, 0);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_data[0], offset, 3);
		return returnVal;
	}
	XsVector_assign(returnVal, 0, 0);
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains calibrated magnetometer data
  \returns true if this packet contains calibrated magnetometer data
*/
int XsDataPacket_containsCalibratedMagneticField(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_MagneticField) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update calibrated magnetometer data for the item
*/
void XsDataPacket_setCalibratedMagneticField(XsDataPacket* thisPtr, const XsVector* vec)
{
	const int numValues = 3;

	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_MagneticField | XDI_SubFormatDouble) == -1)
	{
		// add
		//m_msg.m_autoUpdateChecksum = false;

		XsSize dsz = numValues*XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+dsz);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_MagneticField | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) dsz, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &vec->m_data[0], XsDataPacket_itemOffsetExact(thisPtr, XDI_MagneticField | XDI_SubFormatDouble), numValues);
}

/*! \relates XsDataPacket
	\brief Return the calibrated Data component of a data item.
	\param returnVal Storage for the requested data
	\returns Returns the supplied \a returnVal filled with the requested data
*/
XsCalibratedData* XsDataPacket_calibratedData(const XsDataPacket* thisPtr, XsCalibratedData* returnVal)
{
	assert(returnVal);
	//double* bare = (double*) buffer.m_acc.m_data;
	if (XsDataPacket_containsCalibratedAcceleration(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_Acceleration);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, returnVal->m_acc.m_vector.m_data, offset, 3);
	}
	else
		memset(returnVal->m_acc.m_vector.m_data, 0, 3*sizeof(XsReal));

	if (XsDataPacket_containsCalibratedGyroscopeData(thisPtr)) {
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_RateOfTurn);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, returnVal->m_gyr.m_vector.m_data, offset, 3);
	}
	else
		memset(returnVal->m_gyr.m_vector.m_data, 0, 3*sizeof(XsReal));

	if (XsDataPacket_containsCalibratedMagneticField(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_MagneticField);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, returnVal->m_mag.m_vector.m_data, offset, 3);
	}
	else
		memset(returnVal->m_mag.m_vector.m_data, 0, 3*sizeof(XsReal));

	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains calibrated Data
	\returns Returns whether the packet contains calibrated data or not
	\note Calibrated data is only present if *all* components are present.
*/
int XsDataPacket_containsCalibratedData(const XsDataPacket* thisPtr)
{
	// Note: calibrated data is only present if *all* components are present
	return (!(XsDataPacket_itemOffsetLoose(thisPtr, XDI_Acceleration) == -1 || XsDataPacket_itemOffsetLoose(thisPtr, XDI_RateOfTurn) == -1 || XsDataPacket_itemOffsetLoose(thisPtr, XDI_MagneticField) == -1));
}

/*! \relates XsDataPacket
	\brief Add/update calibrated Data for the item */
void XsDataPacket_setCalibratedData(XsDataPacket* thisPtr, const XsCalibratedData* data)
{
	uint8_t ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_Acceleration | XDI_SubFormatDouble) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+3*ds);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_Acceleration | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 3*ds, sz+2);
		++thisPtr->m_itemCount;
	}
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_RateOfTurn | XDI_SubFormatDouble) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+3*ds);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RateOfTurn | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 3*ds, sz+2);
		++thisPtr->m_itemCount;
	}
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_MagneticField | XDI_SubFormatDouble) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+3*ds);
		XsMessage_setDataShort(&thisPtr->m_msg, XDI_MagneticField | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 3*ds, sz+2);
		++thisPtr->m_itemCount;
	}

	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, data->m_acc.m_vector.m_data, XsDataPacket_itemOffsetExact(thisPtr, XDI_Acceleration | XDI_SubFormatDouble), 3);
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, data->m_gyr.m_vector.m_data, XsDataPacket_itemOffsetExact(thisPtr, XDI_RateOfTurn | XDI_SubFormatDouble), 3);
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, data->m_mag.m_vector.m_data, XsDataPacket_itemOffsetExact(thisPtr, XDI_MagneticField | XDI_SubFormatDouble), 3);
}

/*! \cond XS_INTERNAL */
/*!	\relates XsDataPacket
	\brief Check if data item contains quaternion orientation data
	\returns true if this packet contains quaternion orientation data
*/
int XsDataPacket_containsOrientationQuaternion(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_Quaternion) != -1);
}

/*! \relates XsDataPacket
	\brief Check if data item contains euler orientation data
	\returns true if this packet contains euler orientation data
*/
int XsDataPacket_containsOrientationEuler(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_EulerAngles) != -1);
}

/*! \relates XsDataPacket
	\brief Check if data item contains matrix orientation data
	\returns true if this packet contains matrix orientation data
*/
int XsDataPacket_containsOrientationMatrix(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_RotationMatrix) != -1);
}

/*! \brief Returns quaternion that defines the rotation necessary to rotate from coordinate system
	\a actual to \a desired when used as the left side in a quaternion multiplication
*/
XsQuaternion preRotFromXdi(XsDataIdentifier actual, XsDataIdentifier desired)
{
	static const XsQuaternion q_id = {{{1, 0, 0, 0}}};										// x -> x
	static const XsQuaternion q_nwu2ned = {{{0, 1, 0, 0}}};									// nwu -> ned = 180 degrees x
	static const XsQuaternion q_enu2nwu = {{{1.4142135623730950488016887242097*0.5, 0, 0, -1.4142135623730950488016887242097*0.5}}};	// enu -> nwu = 90 degrees z
	static const XsQuaternion q_enu2ned = {{{0, -1.4142135623730950488016887242097*0.5, -1.4142135623730950488016887242097*0.5, 0}}};	// enu -> ned = 90 degrees z followed by 180 degrees x

	static const XsQuaternion q_ned2nwu = {{{0, -1, 0, 0}}};
	static const XsQuaternion q_nwu2enu = {{{1.4142135623730950488016887242097*0.5, 0, 0, 1.4142135623730950488016887242097*0.5}}};
	static const XsQuaternion q_ned2enu = {{{0, 1.4142135623730950488016887242097*0.5, 1.4142135623730950488016887242097*0.5, 0}}};

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

/*! \relates XsDataPacket
	\brief Return the orientation component of a data item as a quaternion.

	\param returnVal An %XsQuaternion to put the requested orientation in
	\param coordinateSystem The coordinate system of the requested orientation. If this does not match
	the stored coordinate system, it will be transformed to the requested orientation.

	\returns An XsQuaternion containing the orientation data
*/
XsQuaternion* XsDataPacket_orientationQuaternion(const XsDataPacket* thisPtr, XsQuaternion* returnVal, XsDataIdentifier coordinateSystem)
{
	assert(returnVal);
	if (XsDataPacket_containsOrientationQuaternion(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_Quaternion);
		XsDataIdentifier foundCsys = (XsDataIdentifier) XsMessage_getDataShort(&thisPtr->m_msg, offset-3);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, returnVal->m_data, offset, 4);
		if ((coordinateSystem & XDI_CoordSysMask) != (foundCsys & XDI_CoordSysMask))
		{
			XsQuaternion rot;
			rot = preRotFromXdi(foundCsys, coordinateSystem);
			XsQuaternion_multiply(&rot, returnVal, returnVal);
		}
	}
	else if (XsDataPacket_containsOrientationMatrix(thisPtr))
	{
		XsMatrix3x3 m;
		XsMatrix3x3_construct(&m);
		XsQuaternion_fromRotationMatrix(returnVal, XsDataPacket_orientationMatrix(thisPtr, &m.m_matrix, coordinateSystem));
	}
	else if (XsDataPacket_containsOrientationEuler(thisPtr))
	{
		XsEuler eul;
		XsQuaternion_fromEulerAngles(returnVal, XsDataPacket_orientationEuler(thisPtr, &eul, coordinateSystem));
	}
	else
		memset(returnVal->m_data, 0, 4*sizeof(XsReal));

	return returnVal;
}

/*! \relates XsDataPacket
	\brief Add/update quaternion orientation Data for the item */
void XsDataPacket_setOrientationQuaternion(XsDataPacket* thisPtr, const XsQuaternion* data, XsDataIdentifier coordinateSystem)
{
	const int numValues = 4;
	int offset = XsDataPacket_itemOffsetMasked(thisPtr, XDI_Quaternion | XDI_SubFormatDouble, ~XDI_CoordSysMask);	//lint !e64

	if (offset == -1)
	{
		// add
		XsSize dsz = numValues*XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+dsz);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_Quaternion | XDI_SubFormatDouble | (coordinateSystem & XDI_CoordSysMask), sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) dsz, sz+2);
		++thisPtr->m_itemCount;

		offset = (int) sz+3;
	}
	else
	{
		uint16_t csys = XsMessage_getDataShort(&thisPtr->m_msg, offset-3);
		if ((csys & XDI_CoordSysMask) != (coordinateSystem & XDI_CoordSysMask))
			XsMessage_setDataShort(&thisPtr->m_msg, (csys & ~XDI_CoordSysMask) | (coordinateSystem & XDI_CoordSysMask), offset-3);
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &data->m_data[0], offset, numValues);
}

/*! \relates XsDataPacket
	\brief Return the orientation component of a data item as a euler angles.

	\param returnVal An %XsEuler to put the requested orientation in
	\param coordinateSystem The coordinate system of the requested orientation. If this does not match
	the stored coordinate system, it will be transformed to the requested orientation.

	\returns A %XsEuler containing the orientation data
*/
XsEuler* XsDataPacket_orientationEuler(const XsDataPacket* thisPtr, XsEuler* returnVal, XsDataIdentifier coordinateSystem)
{
	assert(returnVal);
	if (XsDataPacket_containsOrientationEuler(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_EulerAngles);
		XsDataIdentifier foundCsys = (XsDataIdentifier) XsMessage_getDataShort(&thisPtr->m_msg, offset-3);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_x, offset, 3);
		if ((coordinateSystem & XDI_CoordSysMask) != (foundCsys & XDI_CoordSysMask))
		{
			XsQuaternion rot, q;
			rot = preRotFromXdi(foundCsys, coordinateSystem);
			XsQuaternion_fromEulerAngles(&q, returnVal);
			XsQuaternion_multiply(&rot, &q, &q);
			XsEuler_fromQuaternion(returnVal, &q);
		}
	}
	else if (XsDataPacket_containsOrientationQuaternion(thisPtr))
	{
		XsQuaternion q;
		XsEuler_fromQuaternion(returnVal, XsDataPacket_orientationQuaternion(thisPtr, &q, coordinateSystem));
	}
	else if (XsDataPacket_containsOrientationMatrix(thisPtr))
	{
		XsQuaternion q;
		XsMatrix3x3 m;
		XsMatrix3x3_construct(&m);
		XsQuaternion_fromRotationMatrix(&q, XsDataPacket_orientationMatrix(thisPtr, &m.m_matrix, coordinateSystem));
		XsEuler_fromQuaternion(returnVal, &q);
	}
	else
	{
		returnVal->m_x = XsMath_zero;
		returnVal->m_y = XsMath_zero;
		returnVal->m_z = XsMath_zero;
	}
	return returnVal;
}


/*! \relates XsDataPacket
	\brief Add/update quaternion orientation Data for the item */
void XsDataPacket_setOrientationEuler(XsDataPacket* thisPtr, const XsEuler* data, XsDataIdentifier coordinateSystem)
{
	const int numValues = 3;

	int offset = XsDataPacket_itemOffsetMasked(thisPtr, XDI_EulerAngles | XDI_SubFormatDouble, ~XDI_CoordSysMask);	//lint !e64
	if (offset == -1)
	{
		// add
		XsSize dsz = numValues*XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+dsz);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_EulerAngles | XDI_SubFormatDouble | (coordinateSystem & XDI_CoordSysMask), sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) dsz, sz+2);
		++thisPtr->m_itemCount;

		offset = (int) sz+3;
	}
	else
	{
		uint16_t csys = XsMessage_getDataShort(&thisPtr->m_msg, offset-3);
		if ((csys & XDI_CoordSysMask) != (coordinateSystem & XDI_CoordSysMask))
			XsMessage_setDataShort(&thisPtr->m_msg, (csys & ~XDI_CoordSysMask) | (coordinateSystem & XDI_CoordSysMask), offset-3);
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &data->m_x, offset, numValues);
}

/*! \relates XsDataPacket
	\brief Return the orientation component of a data item as a orientation matrix.

	\param returnVal An %XsMatrix to put the requested orientation in
	\param coordinateSystem The coordinate system of the requested orientation. If this does not match
	the stored coordinate system, it will be transformed to the requested orientation.

	\returns An %XsMatrix containing the orientation data
*/
XsMatrix* XsDataPacket_orientationMatrix(const XsDataPacket* thisPtr, XsMatrix* returnVal, XsDataIdentifier coordinateSystem)
{
	assert(returnVal);
	if (XsDataPacket_containsOrientationMatrix(thisPtr))
	{
		uint16_t ds;
		int i, j;
		uint16_t k = 0;
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_RotationMatrix);
		XsDataIdentifier foundCsys = (XsDataIdentifier) XsMessage_getDataShort(&thisPtr->m_msg, offset-3);
		XsMatrix_assign(returnVal, 3, 3, 0, 0, 0);
		// remember to use column major order in the xbus message!
		ds = XsDataPacket_getFPValueSize(thisPtr->m_lastFoundId);
		for (i=0 ; i<3 ; ++i)
		{
			for (j=0 ; j<3 ; ++j, k+=ds)
			{
				double tmp;
				XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &tmp, offset+k, 1);
				XsMatrix_setValue(returnVal, j, i, tmp);
			}
		}
		if ((coordinateSystem & XDI_CoordSysMask) != (foundCsys & XDI_CoordSysMask))
		{
			XsQuaternion rot, q;
			rot = preRotFromXdi(foundCsys, coordinateSystem);
			XsQuaternion_fromRotationMatrix(&q, returnVal);
			XsQuaternion_multiply(&rot, &q, &q);
			XsMatrix_fromQuaternion(returnVal, &q);
		}
	}
	else if (XsDataPacket_containsOrientationQuaternion(thisPtr))
	{
		XsQuaternion q;
		XsMatrix_fromQuaternion(returnVal, XsDataPacket_orientationQuaternion(thisPtr, &q, coordinateSystem));
	}
	else if (XsDataPacket_containsOrientationEuler(thisPtr))
	{
		XsEuler eul;
		XsQuaternion q;
		XsQuaternion_fromEulerAngles(&q, XsDataPacket_orientationEuler(thisPtr, &eul, coordinateSystem));
		XsMatrix_fromQuaternion(returnVal, &q);
	}
	else
		XsMatrix_destruct(returnVal);
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Add/update quaternion orientation Data for the item */
void XsDataPacket_setOrientationMatrix(XsDataPacket* thisPtr, const XsMatrix* data, XsDataIdentifier coordinateSystem)
{
	const int numValues = 9;
	XsSize i,j,k = 0;
	XsSize ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);

	// remember to use column major order in the xbus message!
	int offset = XsDataPacket_itemOffsetMasked(thisPtr, XDI_RotationMatrix | XDI_SubFormatDouble, ~XDI_CoordSysMask);	//lint !e64
	if (offset == -1)
	{
		// add
		XsSize dsz = numValues*XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+dsz);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_RotationMatrix | XDI_SubFormatDouble | (coordinateSystem & XDI_CoordSysMask), sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) dsz, sz+2);
		++thisPtr->m_itemCount;

		offset = (int) sz+3;
	}
	else
	{
		uint16_t csys = XsMessage_getDataShort(&thisPtr->m_msg, offset-3);
		if ((csys & XDI_CoordSysMask) != (coordinateSystem & XDI_CoordSysMask))
			XsMessage_setDataShort(&thisPtr->m_msg, (csys & ~XDI_CoordSysMask) | (coordinateSystem & XDI_CoordSysMask), offset-3);
	}
	// update
	for (i=0 ; i<3 ; ++i)
	{
		for (j=0 ; j<3 ; ++j, k += ds)
		{
			double tmp = XsMatrix_value(data, j, i);
			XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &tmp, offset+k, 1);
		}
	}
}

/*! \relates XsDataPacket
	\brief Check if data item contains orientation Data of any kind
	\returns true if this packet contains orientation data
*/
int XsDataPacket_containsOrientation(const XsDataPacket* thisPtr)
{
	return XsDataPacket_itemOffsetMasked(thisPtr, XDI_OrientationGroup, XDI_TypeMask) != -1;
}

/*! \relates XsDataPacket
	\brief Returns the data identifier of the first orientation data of any kind in the packet
	\returns The %XsDataIdentifier of the first orientation data of any kind in the packet
*/
XsDataIdentifier XsDataPacket_orientationIdentifier(const XsDataPacket* thisPtr)
{
	int offset = XsDataPacket_itemOffsetMasked(thisPtr, XDI_OrientationGroup, XDI_TypeMask);
	if (offset == -1)
		return XDI_None;
	return (XsDataIdentifier) XsMessage_getDataShort(&thisPtr->m_msg, offset-3);
}

/*! \relates XsDataPacket
	\brief Returns the coordinate system of the first orientation data of any kind in the packet
	\returns The XsDataIdentifier of the coordinate system of the first orientation data of any kind in the packet
*/
XsDataIdentifier XsDataPacket_coordinateSystemOrientation(const XsDataPacket* thisPtr)
{
	return XsDataPacket_orientationIdentifier(thisPtr) & XDI_CoordSysMask;
}

/*! \relates XsDataPacket
	\brief The gps PVT data component of a data item.

	\param returnVal : An XsGpsPvtData object to put the requested in

	\returns An XsGpsPvtData containing the gps PVT data
*/
XsGpsPvtData* XsDataPacket_gpsPvtData(const XsDataPacket* thisPtr, XsGpsPvtData* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsGpsPvtData(thisPtr))
	{
		XsSize i;
		uint32_t *bareln;
		returnVal->m_pressureAge = XsMessage_getDataByte(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_PressureAge));
		returnVal->m_gpsAge = XsMessage_getDataByte(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsAge));

		returnVal->m_pressure = XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_BaroPressure));

		returnVal->m_itow = XsMessage_getDataLong(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_Itow));
		//lint --e{661, 662} these fields are named first-of-array fields
		bareln = (uint32_t*) &returnVal->m_latitude;
		for (i = 0; i < 3; i++)
			bareln[i] = XsMessage_getDataLong(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSol) + (4*i) + 12); //! \todo data is now ECEF --> convert to LLH

		bareln = (uint32_t*)&returnVal->m_veln;
		for (i = 0; i < 3; i++)
			bareln[i] = XsMessage_getDataLong(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSol) + (4*i) + 28); //! \todo convert from ECEF to LLH

		bareln = (uint32_t*)&returnVal->m_hacc;
		for (i = 0; i < 3; i++)
			bareln[i] = XsMessage_getDataLong(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSol) + 24);   //! \todo determine/convert this correctly

	}
	else
		XsGpsPvtData_destruct(returnVal);
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains gps PVT data
  \returns true if this packet contains gps PVT data
*/
int XsDataPacket_containsGpsPvtData(const XsDataPacket* thisPtr)
{
	return !((XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsAge) == -1) ||
			(XsDataPacket_itemOffsetExact(thisPtr, XDI_PressureAge) == -1) ||
			(XsDataPacket_itemOffsetExact(thisPtr, XDI_Itow) == -1));
}

/*! \relates XsDataPacket
	\brief Add/update gps PVT data for the item

	\param data : The data to update the XsDataPacket with
*/
void XsDataPacket_setGpsPvtData(XsDataPacket* thisPtr, const XsGpsPvtData* data)
{
	XsSize i;
	uint32_t *bareln;

	if (!XsDataPacket_containsGpsPvtData(thisPtr))
	{
		// add
		//XsSize dsz = numValues*XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+5*3+60);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_GpsAge, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 1, sz+2);
		//XsMessage_resizeData(&thisPtr->m_msg, XsMessage_dataSize(&thisPtr->m_msg) + 1);
		sz += 4;

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_PressureAge, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 1, sz+2);
		//XsMessage_resizeData(&thisPtr->m_msg, XsMessage_dataSize(&thisPtr->m_msg) + 1);
		sz += 4;

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_BaroPressure, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, sizeof(uint16_t), sz+2);
		//XsMessage_resizeData(&thisPtr->m_msg, XsMessage_dataSize(&thisPtr->m_msg) + sizeof(uint16_t));
		sz += 5;

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_Itow, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, sizeof(uint32_t), sz+2);
		//XsMessage_resizeData(&thisPtr->m_msg, XsMessage_dataSize(&thisPtr->m_msg) + sizeof(uint32_t));
		sz += 7;

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_GpsSol, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 52, sz+2);
		//XsMessage_resizeData(&thisPtr->m_msg, XsMessage_dataSize(&thisPtr->m_msg) + 52);

		thisPtr->m_itemCount += 5;
	}
	// update

	XsMessage_setDataByte(&thisPtr->m_msg, data->m_pressureAge, XsDataPacket_itemOffsetExact(thisPtr, XDI_PressureAge));
	XsMessage_setDataByte(&thisPtr->m_msg, data->m_gpsAge, XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsAge));

	XsMessage_setDataShort(&thisPtr->m_msg, data->m_pressure, XsDataPacket_itemOffsetExact(thisPtr, XDI_BaroPressure));

	XsMessage_setDataLong(&thisPtr->m_msg, data->m_itow, XsDataPacket_itemOffsetExact(thisPtr, XDI_Itow));

	//lint --e{661, 662} these fields are named first-of-array fields
	//! \todo convert the LLH based input back to ECEF to create a correct GPS SOL packet
	bareln = (uint32_t*) &data->m_latitude;
	for (i = 0; i < 3; i++)
		XsMessage_setDataLong(&thisPtr->m_msg, bareln[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSol) + 12 + (4*i));

	bareln = (uint32_t*)&data->m_veln;
	for (i = 0; i < 3; i++)
		XsMessage_setDataLong(&thisPtr->m_msg, bareln[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSol) + 28 + (4*i));

	bareln = (uint32_t*)&data->m_hacc;
	for (i = 0; i < 3; i++)
		XsMessage_setDataLong(&thisPtr->m_msg, bareln[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSol) + 24);
}

/*! \relates XsDataPacket
	\brief Check if data item contains pressure data
  \returns true if this packet contains pressure data
*/
int XsDataPacket_containsPressure(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_BaroPressure) != -1);
}

/*! \relates XsDataPacket
	\brief Check if data item contains pressure age data
  \returns true if this packet contains pressure age data
*/
int XsDataPacket_containsPressureAge(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_PressureAge) != -1);
}

/*! \relates XsDataPacket
	\brief The air pressure component of a data item.

	\param returnVal : An XsPressure object to put the requested in

	\returns An XsPressure object containing the pressure and if available the pressure age
*/
XsPressure* XsDataPacket_pressure(const XsDataPacket* thisPtr, XsPressure* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsPressure(thisPtr))
	{
		returnVal->m_pressure = XsMessage_getDataLong(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_BaroPressure));
		returnVal->m_pressureAge = 0;
	}
	if (XsDataPacket_containsPressureAge(thisPtr))
	{
		returnVal->m_pressureAge = XsMessage_getDataByte(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_PressureAge));
	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Add/update pressure data for the item
*/
void XsDataPacket_setPressure(XsDataPacket* thisPtr, const XsPressure* data)
{
	if (!XsDataPacket_containsPressure(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+11);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_BaroPressure, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 4, sz+2);
		//XsMessage_resizeData(&thisPtr->m_msg, XsMessage_dataSize(&thisPtr->m_msg) + 4);
		sz += 7;

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_PressureAge, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 1, sz+2);
		//XsMessage_resizeData(&thisPtr->m_msg, XsMessage_dataSize(&thisPtr->m_msg) + 1);

		thisPtr->m_itemCount += 2;
	}
	// update
	XsMessage_setDataLong(&thisPtr->m_msg, (uint32_t)data->m_pressure, XsDataPacket_itemOffsetExact(thisPtr, XDI_BaroPressure));
	XsMessage_setDataByte(&thisPtr->m_msg, data->m_pressureAge, XsDataPacket_itemOffsetExact(thisPtr, XDI_PressureAge));
}

/*! \relates XsDataPacket
	\brief Return the strapdown integration data component of a data item.
	\param returnVal Storage for the requested data
	\returns Returns the supplied \a returnVal filled with the requested data
*/
XsSdiData* XsDataPacket_sdiData(const XsDataPacket* thisPtr, XsSdiData* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsSdiData(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_DeltaQ);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_orientationIncrement.m_data[0], offset, 4);
		offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_DeltaV);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_velocityIncrement.m_vector.m_data[0], offset, 3);
	}
	else
		XsSdiData_destruct(returnVal);
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains strapdown integration data
	\returns Returns true if this packet contains sdi data
*/
int XsDataPacket_containsSdiData(const XsDataPacket* thisPtr)
{
	return !(XsDataPacket_itemOffsetLoose(thisPtr, XDI_DeltaQ) == -1 || XsDataPacket_itemOffsetLoose(thisPtr, XDI_DeltaV) == -1);
}

/*! \relates XsDataPacket
	\brief Add/update strapdown integration data for the item
	\param data The updated data
*/
void XsDataPacket_setSdiData(XsDataPacket* thisPtr, const XsSdiData* data)
{
	{
		int qoff = XsDataPacket_itemOffsetLoose(thisPtr, XDI_DeltaQ);
		XsDataIdentifier dt = XDI_SubFormatDouble;
		if (qoff == -1)
		{
			// add
			XsSize ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
			XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
			XsMessage_resizeData(&thisPtr->m_msg, sz+3+4*ds);

			XsMessage_setDataShort(&thisPtr->m_msg, XDI_DeltaQ | XDI_SubFormatDouble, sz);
			XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) ds*4, sz+2);
			++thisPtr->m_itemCount;

			qoff = (int)sz + 3;
		}
		else
			dt = thisPtr->m_lastFoundId;
		// update
		XsMessage_setDataFPValuesById(&thisPtr->m_msg, dt, &data->m_orientationIncrement.m_data[0], qoff, 4);
	}

	{
		int voff = XsDataPacket_itemOffsetLoose(thisPtr, XDI_DeltaV);
		XsDataIdentifier dt = XDI_SubFormatDouble;
		if (voff == -1)
		{
			// add
			XsSize ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
			XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
			XsMessage_resizeData(&thisPtr->m_msg, sz+3+3*ds);

			XsMessage_setDataShort(&thisPtr->m_msg, XDI_DeltaV | XDI_SubFormatDouble, sz);
			XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) ds*3, sz+2);
			++thisPtr->m_itemCount;

			voff = (int)sz + 3;
		}
		else
			dt = thisPtr->m_lastFoundId;

		// update
		XsMessage_setDataFPValuesById(&thisPtr->m_msg, dt, &data->m_velocityIncrement.m_vector.m_data[0], voff, 3);
	}
}

/*! \relates XsDataPacket
	\brief The temperature component of a data item.

	\returns A double containing the temperature value, -1000.0 if the packet does not contain temperature
*/
double XsDataPacket_temperature(const XsDataPacket* thisPtr)
{
	double temp = -1000.0;
	int offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_Temperature);
	if (offset >= 0)
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &temp, offset, 1);
	return temp;
}

/*! \relates XsDataPacket
	\brief Check if data item contains temperature data
  \returns true if this packet contains temperature data
*/
int XsDataPacket_containsTemperature(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_Temperature) != -1);
}

/*! \relates XsDataPacket
	\brief Adds or updates the temperature data in the datapacket

	\details The \a temp is added to the datapacket. If the packet already contains
			 temperature it is replaced with the new value.

	\param temperature : The temperature to set
*/
void XsDataPacket_setTemperature(XsDataPacket* thisPtr, double temperature)
{
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_Temperature | XDI_SubFormatDouble) == -1)
	{
		// add
		XsSize ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+ds);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_Temperature | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) ds, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &temperature, XsDataPacket_itemOffsetExact(thisPtr, XDI_Temperature | XDI_SubFormatDouble), 1);
}

/*! \relates XsDataPacket
	\brief The analog in 1 component of a data item.

	\param returnVal : The XsAnalogInData object that the analog in 1 value will be assigned to

	\returns An XsAnalogInData containing the analog in 1 value
*/
XsAnalogInData* XsDataPacket_analogIn1Data(const XsDataPacket* thisPtr, XsAnalogInData* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsAnalogIn1Data(thisPtr))
		returnVal->m_data = XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_AnalogIn1));
	else
		returnVal->m_data = 0;

	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains analog in 1 data
  \returns true if this packet contains analog in 1 data
*/
int XsDataPacket_containsAnalogIn1Data(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_AnalogIn1) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update analog in 1 data for the item
*/
void XsDataPacket_setAnalogIn1Data(XsDataPacket* thisPtr, const XsAnalogInData* data)
{
	if (!XsDataPacket_containsAnalogIn1Data(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+5);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_AnalogIn1, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 2, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataShort(&thisPtr->m_msg, data->m_data, XsDataPacket_itemOffsetExact(thisPtr, XDI_AnalogIn1));
}

/*! \relates XsDataPacket
	\brief The analog in 2 component of a data item.

	\param returnVal : The XsAnalogInData object that the analog in 2 value will be assigned to

	\returns An XsAnalogInData containing the analog in 2 value
*/
XsAnalogInData* XsDataPacket_analogIn2Data(const XsDataPacket* thisPtr, XsAnalogInData* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsAnalogIn2Data(thisPtr))
		returnVal->m_data = XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_AnalogIn2));
	else
		returnVal->m_data = 0;

	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains analog in 2 data
  \returns true if this packet contains analog in 2 data
*/
int XsDataPacket_containsAnalogIn2Data(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_AnalogIn2) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update analog in 2 data for the item
*/
void XsDataPacket_setAnalogIn2Data(XsDataPacket* thisPtr, const XsAnalogInData* data)
{
	if (!XsDataPacket_containsAnalogIn2Data(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+5);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_AnalogIn2, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 2, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataShort(&thisPtr->m_msg, data->m_data, XsDataPacket_itemOffsetExact(thisPtr, XDI_AnalogIn2));
}


/*! \relates XsDataPacket
	\brief The position lat lon alt component of a data item.

	\param returnVal : The XsVector to return the requested data in

	\returns An XsVector containing the latitude, longitude and altitude values in that order
*/
XsVector* XsDataPacket_positionLLA(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	int offsetLL = XsDataPacket_itemOffsetLoose(thisPtr, XDI_LatLon);
	int offsetA = XsDataPacket_itemOffsetLoose(thisPtr, XDI_AltitudeEllipsoid);
	assert(returnVal);

	if (offsetLL == -1 || offsetA == -1)
		XsVector_destruct(returnVal);
	else
	{
		XsVector_assign(returnVal, 3, 0);
		offsetLL = XsDataPacket_itemOffsetLoose(thisPtr, XDI_LatLon);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_data[0], offsetLL, 2);
		offsetA = XsDataPacket_itemOffsetLoose(thisPtr, XDI_AltitudeEllipsoid);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_data[2], offsetA, 1);
	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains position lat lon alt data
	\returns true if this packet contains position lat lon alt data
*/
int XsDataPacket_containsPositionLLA(const XsDataPacket* thisPtr)
{
	return	(XsDataPacket_itemOffsetLoose(thisPtr, XDI_LatLon) != -1) &&
			(XsDataPacket_itemOffsetLoose(thisPtr, XDI_AltitudeEllipsoid) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update position lat lon alt data for the item

	\param data : The XsVector that conrtains the Lat/Long/Alt data to store in the packet
*/
void XsDataPacket_setPositionLLA(XsDataPacket* thisPtr, const XsVector* data)
{
	uint8_t ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_LatLon | XDI_SubFormatDouble) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+2*ds);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_LatLon | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 2*ds, sz+2);
		++thisPtr->m_itemCount;
	}
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_AltitudeEllipsoid | XDI_SubFormatDouble) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+ds);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_AltitudeEllipsoid | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, ds, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &data->m_data[0], XsDataPacket_itemOffsetExact(thisPtr, XDI_LatLon | XDI_SubFormatDouble), 2);
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &data->m_data[2], XsDataPacket_itemOffsetExact(thisPtr, XDI_AltitudeEllipsoid | XDI_SubFormatDouble), 1);
}

/*! \relates XsDataPacket
	\brief The position latitude longitude component of a data item.

	\param returnVal : The XsVector to return the requested data in

	\returns An XsVector containing the latitude and longitude values in that order
	\sa XsDataPacket_containsLatitudeLongitude
	\sa XsDataPacket_positionLLA \sa XsDataPacket_altitude
*/
XsVector* XsDataPacket_latitudeLongitude(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	int offsetLL = XsDataPacket_itemOffsetLoose(thisPtr, XDI_LatLon);
	assert(returnVal);

	if (offsetLL == -1)
		XsVector_destruct(returnVal);
	else
	{
		XsVector_assign(returnVal, 2, 0);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_data[0], offsetLL, 2);
	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains position latitude longitude data
	\returns true if this packet contains position latitude longitude data
	\sa XsDataPacket_containsPositionLLA \sa XsDataPacket_containsAltitude
*/
int XsDataPacket_containsLatitudeLongitude(const XsDataPacket* thisPtr)
{
	return XsDataPacket_itemOffsetLoose(thisPtr, XDI_LatLon) != -1;
}

/*! \relates XsDataPacket
	\brief Add/update position latitude longitude data for the item

	\param data : The XsVector that contains the latitude longitude data to store in the packet
	\sa XsDataPacket_setPositionLLA \sa XsDataPacket_setAltitude
*/
void XsDataPacket_setLatitudeLongitude(XsDataPacket* thisPtr, const XsVector* data)
{
	uint8_t ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
	int offsetLL = XsDataPacket_itemOffsetExact(thisPtr, XDI_LatLon | XDI_SubFormatDouble);
	assert(data && data->m_size >= 2);
	if (offsetLL == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+2*ds);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_LatLon | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 2*ds, sz+2);
		++thisPtr->m_itemCount;

		offsetLL = (int)(sz + 3);
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &data->m_data[0], offsetLL, 2);
}

/*! \relates XsDataPacket
	\brief The position altitude component of a data item.

	\returns The altitude stored in the packet or XsMath_infinity if no altitude is available
	\sa XsDataPacket_containsAltitude
	\sa XsDataPacket_positionLLA \sa XsDataPacket_latitudeLongitude
*/
double XsDataPacket_altitude(const XsDataPacket* thisPtr)
{
	double rv = XsMath_infinity;
	int offsetA = XsDataPacket_itemOffsetLoose(thisPtr, XDI_AltitudeEllipsoid);

	if (offsetA != -1)
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &rv, offsetA, 1);
	return rv;
}

/*! \relates XsDataPacket
	\brief Check if data item contains position altitude data
	\returns true if this packet contains position altitude data
	\sa XsDataPacket_containsPositionLLA \sa XsDataPacket_containsLatitudeLongitude
*/
int XsDataPacket_containsAltitude(const XsDataPacket* thisPtr)
{
	return	XsDataPacket_itemOffsetLoose(thisPtr, XDI_AltitudeEllipsoid) != -1;
}

/*! \relates XsDataPacket
	\brief Add/update altitude data for the item

	\param data : The altitude data to store in the packet
	\sa XsDataPacket_setPositionLLA \sa XsDataPacket_setLatitudeLongitude
*/
void XsDataPacket_setAltitude(XsDataPacket* thisPtr, double data)
{
	uint8_t ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_AltitudeEllipsoid | XDI_SubFormatDouble) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+ds);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_AltitudeEllipsoid | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, ds, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &data, XsDataPacket_itemOffsetExact(thisPtr, XDI_AltitudeEllipsoid | XDI_SubFormatDouble), 1);
}

/*! \relates XsDataPacket
	\brief  The velocity NWU component of a data item.

	\param returnVal : The XsVector to put the data in
	\param coordinateSystem The coordinate system of the requested velocity. If this does not match
	the stored coordinate system, it will be transformed to the requested velocity.

	\returns A XsVector containing the x, y and z axis values in that order
*/
XsVector* XsDataPacket_velocity(const XsDataPacket* thisPtr, XsVector* returnVal, XsDataIdentifier coordinateSystem)
{
	assert(returnVal);
	if (XsDataPacket_containsVelocity(thisPtr))
	{
		XsReal vel[3];
		XsDataIdentifier desiredCsys, actualCsys;
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_VelocityXYZ);
		XsDataIdentifier foundCsys = (XsDataIdentifier) XsMessage_getDataShort(&thisPtr->m_msg, offset-3);

		XsVector_assign(returnVal, 3, 0);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &vel[0], offset, 3);

		desiredCsys = coordinateSystem & XDI_CoordSysMask;
		actualCsys = foundCsys & XDI_CoordSysMask;

		if (desiredCsys != actualCsys)
		{
			switch (desiredCsys)
			{
				case XDI_CoordSysEnu:
				{
					if (actualCsys == XDI_CoordSysNwu)
					{
						returnVal->m_data[0] = -vel[1];
						returnVal->m_data[1] = vel[0];
						returnVal->m_data[2] = vel[2];
					} else if (actualCsys == XDI_CoordSysNed)
					{
						returnVal->m_data[0] = vel[1];
						returnVal->m_data[1] = vel[0];
						returnVal->m_data[2] = -vel[2];
					}
				} break;

				case XDI_CoordSysNwu:
				{
					if (actualCsys == XDI_CoordSysEnu)
					{
						returnVal->m_data[0] = vel[1];
						returnVal->m_data[1] = -vel[0];
						returnVal->m_data[2] = vel[2];
					} else if (actualCsys == XDI_CoordSysNed)
					{
						returnVal->m_data[0] = vel[0];
						returnVal->m_data[1] = -vel[1];
						returnVal->m_data[2] = -vel[2];
					}
				} break;

				case XDI_CoordSysNed:
				{
					if (actualCsys == XDI_CoordSysEnu)
					{
						returnVal->m_data[0] = vel[1];
						returnVal->m_data[1] = vel[0];
						returnVal->m_data[2] = -vel[2];
					} else if (actualCsys == XDI_CoordSysNwu)
					{
						returnVal->m_data[0] = vel[0];
						returnVal->m_data[1] = -vel[1];
						returnVal->m_data[2] = -vel[2];
					}
				} break;

				default:
				{
					returnVal->m_data[0] = vel[0];
					returnVal->m_data[1] = vel[1];
					returnVal->m_data[2] = vel[2];
				} break;
			}
		} else
		{
			returnVal->m_data[0] = vel[0];
			returnVal->m_data[1] = vel[1];
			returnVal->m_data[2] = vel[2];
		}
	}
	else
		XsVector_destruct(returnVal);
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains velocity NED data
  \returns true if this packet contains velocity NED data
*/
int XsDataPacket_containsVelocity(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_VelocityXYZ) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update velocity NED data for the item
*/
void XsDataPacket_setVelocity(XsDataPacket* thisPtr, const XsVector* data, XsDataIdentifier coordinateSystem)
{
	const int numValues = 3;
	int offset = XsDataPacket_itemOffsetMasked(thisPtr, XDI_VelocityXYZ | XDI_SubFormatDouble, ~XDI_CoordSysMask); //lint !e64
	if (offset == -1)
	{
		// add
		XsSize ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+numValues*ds);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_VelocityXYZ | XDI_SubFormatDouble | (coordinateSystem & XDI_CoordSysMask), sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) (numValues*ds), sz+2);
		++thisPtr->m_itemCount;
	} else
	{
		uint16_t csys = XsMessage_getDataShort(&thisPtr->m_msg, offset - 3);
		if ((csys & XDI_CoordSysMask) != (coordinateSystem & XDI_CoordSysMask))
			XsMessage_setDataShort(&thisPtr->m_msg, (csys & ~XDI_CoordSysMask) | (coordinateSystem & XDI_CoordSysMask), offset-3);
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &data->m_data[0], XsDataPacket_itemOffsetExact(thisPtr, XDI_VelocityXYZ | (coordinateSystem & XDI_CoordSysMask) | XDI_SubFormatDouble), numValues);
}

/*! \relates XsDataPacket
	\brief Returns the data identifier of the first velocity data of any kind in the packet
	\returns The %XsDataIdentifier of the first velocity data of any kind in the packet
*/
XsDataIdentifier XsDataPacket_velocityIdentifier(const XsDataPacket* thisPtr)
{
	int offset = XsDataPacket_itemOffsetMasked(thisPtr, XDI_VelocityGroup, XDI_TypeMask);
	if (offset == -1)
		return XDI_None;
	return (XsDataIdentifier) XsMessage_getDataShort(&thisPtr->m_msg, offset-3);
}

/*! \relates XsDataPacket
	\brief Returns the coordinate system of the first velocity data of any kind in the packet
	\returns The XsDataIdentifier of the coordinate system of the first velocity data of any kind in the packet
*/
XsDataIdentifier XsDataPacket_coordinateSystemVelocity(const XsDataPacket* thisPtr)
{
	return XsDataPacket_velocityIdentifier(thisPtr) & XDI_CoordSysMask;
}

/*! \relates XsDataPacket
	\brief The status component of a data item.

	\returns An uint32_t containing the status value
*/
uint32_t XsDataPacket_status(const XsDataPacket* thisPtr)
{
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusWord) != -1)
		return XsMessage_getDataLong(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusWord));
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusByte) != -1)
		return XsMessage_getDataByte(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusByte));
	return 0;
}

/*! \relates XsDataPacket
	\brief Returns whether the %XsDataPacket contains a statusbyte
	\returns Returns true if this packet contains a statusbyte
*/
int XsDataPacket_containsStatusByte(const XsDataPacket* thisPtr)
{
	return XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusByte) != -1;
}

/*! \relates XsDataPacket
	\brief Check if data item contains legacy status data
  \returns true if this packet contains legacy status data
*/
int XsDataPacket_containsStatus(const XsDataPacket* thisPtr)
{
	return XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusWord) != -1 || XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusByte) != -1;
}

/*! \relates XsDataPacket
	\brief Check if data item contains detailed status data
  \returns true if this packet contains detailed status data
*/
int XsDataPacket_containsDetailedStatus(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusWord) != -1);
}


/*! \relates XsDataPacket
	\brief Add/update status data for the item
*/
void XsDataPacket_setStatusByte(XsDataPacket* thisPtr, uint8_t data)
{
	if (!XsDataPacket_containsStatusByte(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+1);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_StatusByte, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 1, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataByte(&thisPtr->m_msg, data, XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusByte));

	// update status word if present
	if (XsDataPacket_containsDetailedStatus(thisPtr))
	{
		XsMessage_setDataLong(&thisPtr->m_msg, (uint32_t)data, XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusWord));
	}
}

/*! \relates XsDataPacket
	\brief Add/update status data for the item
*/
void XsDataPacket_setStatus(XsDataPacket* thisPtr, uint32_t data)
{
	if (!XsDataPacket_containsDetailedStatus(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+4);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_StatusWord, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 4, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataLong(&thisPtr->m_msg, data, XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusWord));

	// update status byte if present
	if (XsDataPacket_containsStatusByte(thisPtr))
	{
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t)data, XsDataPacket_itemOffsetExact(thisPtr, XDI_StatusByte));
	}
}

/*! \relates XsDataPacket
	\brief Add/update trigger indication data for the item
	\param[in] triggerId The trigger data identifier to add data for (e.g. XDI_TriggerIn1 or XDI_TriggerIn2)
	\param[in] triggerIndicationData pointer the a XsTriggerIndicationData buffer containing the data to set
*/
void XsDataPacket_setTriggerIndication(XsDataPacket* thisPtr, XsDataIdentifier triggerId, XsTriggerIndicationData const * triggerIndicationData)
{
	uint8_t line = 0;
	uint8_t polarity = 0;
	uint32_t timestamp = 0;
	uint16_t frameNumber = 0;
	int offset = 0;

	static const XsSize DATASIZE = 8; // 1 + 1 + 4 + 2

	assert(triggerIndicationData != 0);
	if (!XsDataPacket_containsTriggerIndication(thisPtr, triggerId))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz + 3 + DATASIZE);

		XsMessage_setDataShort(&thisPtr->m_msg, triggerId, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) DATASIZE, sz + 2);
		++thisPtr->m_itemCount;
	}

	// update
	line = triggerIndicationData->m_line;
	polarity = triggerIndicationData->m_polarity;
	timestamp = triggerIndicationData->m_timestamp;
	frameNumber = triggerIndicationData->m_frameNumber;
	offset = XsDataPacket_itemOffsetExact(thisPtr, triggerId);

	XsMessage_setDataByte(&thisPtr->m_msg, line, offset + 0);
	XsMessage_setDataByte(&thisPtr->m_msg, polarity, offset + 1);
	XsMessage_setDataLong(&thisPtr->m_msg, timestamp, offset + 2);
	XsMessage_setDataShort(&thisPtr->m_msg, frameNumber, offset + 6);
}

/*! \relates XsDataPacket
	\brief Returns the trigger indication data of a packet
	If the packet does not contain the requested data, the return val struct will be set to all zeroes
	\param[in] triggerId The trigger data identifier to add data for (e.g. XDI_TriggerIn1 or XDI_TriggerIn2)
	\param[out] returnVal pointer to the trigger indication data of a packet.
	\note returnVal should point to a buffer large enough to hold sizeof(XsTriggerIndicationData) bytes of data
	\returns Returns a pointer to the trigger indication data of a packet
*/
XsTriggerIndicationData* XsDataPacket_triggerIndication(XsDataPacket const * thisPtr, XsDataIdentifier triggerId, XsTriggerIndicationData* returnVal)
{
	int offset = 0;

	assert(returnVal);
	offset = XsDataPacket_itemOffsetExact(thisPtr, triggerId);
	if (offset != -1)
	{
		returnVal->m_line = XsMessage_getDataByte(&thisPtr->m_msg, offset + 0);
		returnVal->m_polarity = XsMessage_getDataByte(&thisPtr->m_msg, offset + 1);
		returnVal->m_timestamp = XsMessage_getDataLong(&thisPtr->m_msg, offset + 2);
		returnVal->m_frameNumber = XsMessage_getDataShort(&thisPtr->m_msg, offset + 6);
	}
	else
	{
		memset(returnVal, 0, sizeof(XsTriggerIndicationData));
	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains trigger indication data
	\param[in] triggerId The trigger data identifier to check (e.g. XDI_TriggerIn1 or XDI_TriggerIn2)
	\returns true if this packet contains trigger indication data
*/
int XsDataPacket_containsTriggerIndication(XsDataPacket const * thisPtr, XsDataIdentifier triggerId)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, triggerId) != -1);
}

/*! \relates XsDataPacket
	\brief Return the 8 bit packet counter of a packet

	\details This function returns an 8 bit packet counter as used by some third party devices

	\returns Returns the 8 bit packet counter of a packet
*/
uint8_t XsDataPacket_packetCounter8(const XsDataPacket* thisPtr)
{
	//lint --e{647}
	int offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_PacketCounter8);
	if (offset != -1)
		return XsMessage_getDataByte(&thisPtr->m_msg, offset);

	return 0;
}

/*! \relates XsDataPacket
	\brief Check if data item contains an 8 bit packet counter
	\returns true if this packet contains an 8 bit packet counter
*/
int XsDataPacket_containsPacketCounter8(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_PacketCounter8) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update 8 bit packet counter data for the item */
void XsDataPacket_setPacketCounter8(XsDataPacket* thisPtr, uint8_t counter)
{
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_PacketCounter8) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+1);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_PacketCounter8, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 1, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataByte(&thisPtr->m_msg, counter, XsDataPacket_itemOffsetExact(thisPtr, XDI_PacketCounter8));
}

/*! \relates XsDataPacket
	\brief Return the packet/frame counter of a packet

	\details For strapdown integration data, this function will return the m_wlastFrameNumber
	For other data, this function will return the m_sc

	This way there is a function that will always return the counter of a packet

	\returns Returns the packet/frame counter of a packet
*/
uint16_t XsDataPacket_packetCounter(const XsDataPacket* thisPtr)
{
	//lint --e{647}
	int offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_PacketCounter);
	if (offset != -1)
		return XsMessage_getDataShort(&thisPtr->m_msg, offset);

	offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_FrameRange);
	if (offset != -1)
		return XsMessage_getDataShort(&thisPtr->m_msg, offset+2);

	return 0;
}

/*! \relates XsDataPacket
	\brief Check if data item contains a packet counter
	\returns true if this packet contains a packet counter
*/
int XsDataPacket_containsPacketCounter(const XsDataPacket* thisPtr)
{
	return ((XsDataPacket_itemOffsetExact(thisPtr, XDI_PacketCounter) != -1) || (XsDataPacket_itemOffsetExact(thisPtr, XDI_FrameRange) != -1));
}

/*! \relates XsDataPacket
	\brief Add/update packet counter data for the item */
void XsDataPacket_setPacketCounter(XsDataPacket* thisPtr, uint16_t counter)
{
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_PacketCounter) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+2);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_PacketCounter, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 2, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataShort(&thisPtr->m_msg, counter, XsDataPacket_itemOffsetExact(thisPtr, XDI_PacketCounter));
}

/*! \relates XsDataPacket
	\brief Return the fine sample time of a packet

	\returns Returns the fine sample time of a packet
*/
uint32_t XsDataPacket_sampleTimeFine(const XsDataPacket* thisPtr)
{
	int offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeFine);
	if (offset != -1)
		return XsMessage_getDataLong(&thisPtr->m_msg, offset);

	return 0;
}

/*! \relates XsDataPacket
	\brief Check if data item XsDataPacket_contains a sample time fine
	\returns true if this packet XsDataPacket_contains a sample time fine
*/
int XsDataPacket_containsSampleTimeFine(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeFine) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update sample tine fine data for the item */
void XsDataPacket_setSampleTimeFine(XsDataPacket* thisPtr, uint32_t counter)
{
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeFine) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+4);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_SampleTimeFine, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 4, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataLong(&thisPtr->m_msg, counter, XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeFine));
}

/*! \relates XsDataPacket
	\brief Return the coarse sample time of a packet
*/
uint32_t XsDataPacket_sampleTimeCoarse(const XsDataPacket* thisPtr)
{
	int offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeCoarse);
	if (offset != -1)
		return XsMessage_getDataLong(&thisPtr->m_msg, offset);

	return 0;
}

/*! \relates XsDataPacket
	\brief Check if data item XsDataPacket_contains a sample time coarse
	\returns true if this packet XsDataPacket_contains a sample time coarse
*/
int XsDataPacket_containsSampleTimeCoarse(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeCoarse) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update sample tine coarse data for the item */
void XsDataPacket_setSampleTimeCoarse(XsDataPacket* thisPtr, uint32_t counter)
{
	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeCoarse) == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+4);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_SampleTimeCoarse, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 4, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataLong(&thisPtr->m_msg, counter, XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeCoarse));
}

/*! \relates XsDataPacket
	\brief Return the full 64-bit sample time of a packet, combined from the fine and coarse sample times
*/
uint64_t XsDataPacket_sampleTime64(const XsDataPacket* thisPtr)
{
	int offset;
	uint64_t t64 = 0;
	offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTime64);
	if (offset != -1)
	{
		t64 = ((uint64_t)XsMessage_getDataLong(&thisPtr->m_msg, offset)) << 32;
		t64 += ((uint64_t)XsMessage_getDataLong(&thisPtr->m_msg, offset + 4));
		return t64;
	}

	offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeCoarse);
	if (offset != -1)
	{
		t64 = ((uint64_t) XsMessage_getDataLong(&thisPtr->m_msg, offset)) * 10000;
		offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeFine);
		if (offset != -1)
			t64 += XsMessage_getDataLong(&thisPtr->m_msg, offset) % 10000;
	}
	else
	{
		offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeFine);
		if (offset != -1)
			t64 = XsMessage_getDataLong(&thisPtr->m_msg, offset);
	}

	return t64;
}

/*! \relates XsDataPacket
	\brief Check if data item XsDataPacket contains a full 64-bit sample time
	\returns true if this packet XsDataPacket contains both a fine and coarse sample time
*/
int XsDataPacket_containsSampleTime64(const XsDataPacket* thisPtr)
{
	return ((XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeFine) != -1) && (XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTimeCoarse) != -1))
		|| (XsDataPacket_itemOffsetExact(thisPtr, XDI_SampleTime64) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update sample tine coarse data for the item */
void XsDataPacket_setSampleTime64(XsDataPacket* thisPtr, uint64_t counter)
{
	XsDataPacket_setSampleTimeCoarse(thisPtr, (uint32_t) (counter / 10000));
	XsDataPacket_setSampleTimeFine(thisPtr, (uint32_t) (counter % 10000));
}

/*! \relates XsDataPacket
	\brief The utc time component of a data item.

	\param returnVal : The XsUtcTime to return the requested data in

	\returns An XsUtcTime containing the utc time value
*/
XsUtcTime* XsDataPacket_utcTime(const XsDataPacket* thisPtr, XsUtcTime* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsUtcTime(thisPtr))
	{
		int i;
		uint8_t *bareByte;
		returnVal->m_nano = XsMessage_getDataLong(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_UtcTime));
		returnVal->m_year = XsMessage_getDataShort(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_UtcTime)+4);

		//lint --e{662, 661} these fields are named first-of-array fields
		// month, day, hour, minute, second and valid
		bareByte = (uint8_t*) &returnVal->m_month;
		for (i=0; i < 6; ++i)
			bareByte[i] = XsMessage_getDataByte(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_UtcTime)+ 6 + i);
	}
	else
		memset(returnVal, 0, sizeof(XsUtcTime));
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains utc time data
  \returns true if this packet contains utc time data
*/
int XsDataPacket_containsUtcTime(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_UtcTime) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update utc time data for the item
*/
void XsDataPacket_setUtcTime(XsDataPacket* thisPtr, const XsUtcTime* data)
{
	XsSize i;
	int8_t* bareByte;

	if (!XsDataPacket_containsUtcTime(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+12);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_UtcTime, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 12, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataLong(&thisPtr->m_msg, data->m_nano, XsDataPacket_itemOffsetExact(thisPtr, XDI_UtcTime));
	XsMessage_setDataShort(&thisPtr->m_msg, data->m_year, XsDataPacket_itemOffsetExact(thisPtr, XDI_UtcTime) + 4);

	// month, day, hour, minute, second and valid
	bareByte = (int8_t*)&data->m_month;
	for (i=0; i<6;++i)
		XsMessage_setDataByte(&thisPtr->m_msg, bareByte[i], XsDataPacket_itemOffsetExact(thisPtr, XDI_UtcTime) + 6 + i);	//lint !e661 !e662
}

/*! \relates XsDataPacket
	\brief The free acceleration component of a data item.
	\details Free acceleration is the acceleration with the local gravity vector subtracted.
	\param returnVal : An XsVector to put the requested in

	\returns An XsVector containing the gravity acceleration
*/
XsVector* XsDataPacket_freeAcceleration(const XsDataPacket* thisPtr, XsVector* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsFreeAcceleration(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetLoose(thisPtr, XDI_FreeAcceleration);
		XsVector_assign(returnVal, 3, 0);
		XsMessage_getDataFPValuesById(&thisPtr->m_msg, thisPtr->m_lastFoundId, &returnVal->m_data[0], offset, 3);
	}
	else
		XsVector_destruct(returnVal);
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Check if data item contains free acceleration
	\details Free acceleration is the acceleration with the local gravity vector subtracted.
	\returns true if this packet contains free acceleration
*/
int XsDataPacket_containsFreeAcceleration(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetLoose(thisPtr, XDI_FreeAcceleration) != -1);
}

/*! \relates XsDataPacket
	\brief Add/update free acceleration data for the item
	\details Free acceleration is the acceleration with the local gravity vector subtracted.
	\param g A 3-component vector containing the new free acceleration
*/
void XsDataPacket_setFreeAcceleration(XsDataPacket* thisPtr, const XsVector* g)
{
	const int numValues = 3;

	if (XsDataPacket_itemOffsetExact(thisPtr, XDI_FreeAcceleration | XDI_SubFormatDouble) == -1)
	{
		// add
		XsSize ds = XsDataPacket_getFPValueSize(XDI_SubFormatDouble);
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+numValues*ds);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_FreeAcceleration | XDI_SubFormatDouble, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) (numValues*ds), sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataFPValuesById(&thisPtr->m_msg, XDI_SubFormatDouble, &g->m_data[0], XsDataPacket_itemOffsetExact(thisPtr, XDI_FreeAcceleration | XDI_SubFormatDouble), numValues);
}

/*! \relates XsDataPacket
	\brief Returns the frame range contained in the datapacket

	\param returnVal : The XsRange object that will get the range from the packet

	\returns Returns an XsRange object with the range from the packet
*/
XsRange* XsDataPacket_frameRange(const XsDataPacket* thisPtr, XsRange* returnVal)
{
	assert(returnVal);
	if (XsDataPacket_containsFrameRange(thisPtr))
	{
		int offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_FrameRange);
		returnVal->m_first = XsMessage_getDataShort(&thisPtr->m_msg, offset);
		returnVal->m_last = XsMessage_getDataShort(&thisPtr->m_msg, offset + 2);
	}
	else
	{
		returnVal->m_first = 0;
		returnVal->m_last = 0;
	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Returns whether the datapacket contains a framerange

	\returns Whether the datapacket contains a framerange
*/
int XsDataPacket_containsFrameRange(const XsDataPacket* thisPtr)
{
	return XsDataPacket_itemOffsetExact(thisPtr, XDI_FrameRange) != -1;
}

/*! \relates XsDataPacket
	\brief Sets or updates the frame range in the datapacket

	\param r : The XsRange object that should be added to the packet
*/
void XsDataPacket_setFrameRange(XsDataPacket* thisPtr, const XsRange* r)
{
	int offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_FrameRange);
	if (offset == -1)
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+4);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_FrameRange, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 4, sz+2);
		++thisPtr->m_itemCount;
		offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_FrameRange);
	}
	// update
	//lint --e{647}
	XsMessage_setDataShort(&thisPtr->m_msg, r->m_first, offset);
	XsMessage_setDataShort(&thisPtr->m_msg, r->m_last, offset + 2);
}

/*! \relates XsDataPacket
	\brief Returns the rssi value contained in the datapacket

	\returns Returns the rssi value contained in the datapacket
*/
int XsDataPacket_rssi(const XsDataPacket* thisPtr)
{
	if (XsDataPacket_containsRssi(thisPtr))
		return (int) (int8_t) XsMessage_getDataByte(&thisPtr->m_msg, XsDataPacket_itemOffsetExact(thisPtr, XDI_Rssi));
	else
		return XS_RSSI_UNKNOWN;
}

/*! \relates XsDataPacket
	\brief Returns whether the datapacket contains an rssi value

	\returns Whether the datapacket contains an rssi value
*/
int XsDataPacket_containsRssi(const XsDataPacket* thisPtr)
{
	return XsDataPacket_itemOffsetExact(thisPtr, XDI_Rssi) != -1;
}

/*! \relates XsDataPacket
	\brief Sets or updates the rssi value in the datapacket

	\param r : The rssi value that should be added to the packet
*/
void XsDataPacket_setRssi(XsDataPacket* thisPtr, int r)
{
	if (!XsDataPacket_containsRssi(thisPtr))
	{
		// add
		XsSize sz = XsMessage_dataSize(&thisPtr->m_msg);
		XsMessage_resizeData(&thisPtr->m_msg, sz+3+1);

		XsMessage_setDataShort(&thisPtr->m_msg, XDI_Rssi, sz);
		XsMessage_setDataByte(&thisPtr->m_msg, 1, sz+2);
		++thisPtr->m_itemCount;
	}
	// update
	XsMessage_setDataByte(&thisPtr->m_msg, (uint8_t) (int8_t) r, XsDataPacket_itemOffsetExact(thisPtr, XDI_Rssi));
}

/*! \relates XsDataPacket
	\brief Returns a struct with XsRawGpsDop data
*/
XsRawGpsDop* XsDataPacket_rawGpsDop(const XsDataPacket* thisPtr, XsRawGpsDop* returnVal)
{
	if (XsDataPacket_containsRawGpsDop(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsDop);
		returnVal->m_itow = XsMessage_getDataLong(&thisPtr->m_msg, offset);
		returnVal->m_gdop = XsMessage_getDataShort(&thisPtr->m_msg, offset+4);
		returnVal->m_pdop = XsMessage_getDataShort(&thisPtr->m_msg, offset+6);
		returnVal->m_tdop = XsMessage_getDataShort(&thisPtr->m_msg, offset+8);
		returnVal->m_vdop = XsMessage_getDataShort(&thisPtr->m_msg, offset+10);
		returnVal->m_hdop = XsMessage_getDataShort(&thisPtr->m_msg, offset+12);
		returnVal->m_ndop = XsMessage_getDataShort(&thisPtr->m_msg, offset+14);
		returnVal->m_edop = XsMessage_getDataShort(&thisPtr->m_msg, offset+16);
	}
	else
	{
		memset(returnVal, 0, sizeof(XsRawGpsDop));
	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Returns true if data item contains RawGpsDop, 0 otherwise
*/
int XsDataPacket_containsRawGpsDop(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsDop) != -1);
}

/*! \relates XsDataPacket
	\brief Returns a struct with RawGpsSol data
*/
XsRawGpsSol* XsDataPacket_rawGpsSol(const XsDataPacket* thisPtr, XsRawGpsSol* returnVal)
{
	if (XsDataPacket_containsRawGpsSol(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSol);
		returnVal->m_itow = XsMessage_getDataLong(&thisPtr->m_msg, offset);
		returnVal->m_frac = XsMessage_getDataLong(&thisPtr->m_msg, offset+4);
		returnVal->m_week = XsMessage_getDataShort(&thisPtr->m_msg, offset+8);
		returnVal->m_gpsfix = XsMessage_getDataByte(&thisPtr->m_msg, offset+10);
		returnVal->m_flags = XsMessage_getDataByte(&thisPtr->m_msg, offset+11);
		returnVal->m_ecef_x = XsMessage_getDataLong(&thisPtr->m_msg, offset+12);
		returnVal->m_ecef_y = XsMessage_getDataLong(&thisPtr->m_msg, offset+16);
		returnVal->m_ecef_z = XsMessage_getDataLong(&thisPtr->m_msg, offset+20);
		returnVal->m_pacc = XsMessage_getDataLong(&thisPtr->m_msg, offset+24);
		returnVal->m_ecef_vx = XsMessage_getDataLong(&thisPtr->m_msg, offset+28);
		returnVal->m_ecef_vy = XsMessage_getDataLong(&thisPtr->m_msg, offset+32);
		returnVal->m_ecef_vz = XsMessage_getDataLong(&thisPtr->m_msg, offset+36);
		returnVal->m_sacc = XsMessage_getDataLong(&thisPtr->m_msg, offset+40);
		returnVal->m_pdop = XsMessage_getDataShort(&thisPtr->m_msg, offset+44);
		returnVal->m_res1 = XsMessage_getDataByte(&thisPtr->m_msg, offset+46);
		returnVal->m_numsv = XsMessage_getDataByte(&thisPtr->m_msg, offset+47);
		returnVal->m_res2 = XsMessage_getDataLong(&thisPtr->m_msg, offset+48);
	}
	else
	{
		memset(returnVal, 0, sizeof(XsRawGpsSol));
	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Returns 1 if data item contains RawGpsSol, 0 otherwise
*/
int XsDataPacket_containsRawGpsSol(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSol) != -1);
}

/*! \relates XsDataPacket
	\brief Returns a struct with RawGpsTimeUtc data
*/
XsRawGpsTimeUtc* XsDataPacket_rawGpsTimeUtc(const XsDataPacket* thisPtr, XsRawGpsTimeUtc* returnVal)
{
	if (XsDataPacket_containsRawGpsTimeUtc(thisPtr))
	{
		uint16_t offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsTimeUtc);
		returnVal->m_itow = XsMessage_getDataLong(&thisPtr->m_msg, offset);
		returnVal->m_tacc = XsMessage_getDataLong(&thisPtr->m_msg, offset+4);
		returnVal->m_nano = XsMessage_getDataLong(&thisPtr->m_msg, offset+8);
		returnVal->m_year= XsMessage_getDataShort(&thisPtr->m_msg, offset+12);
		returnVal->m_month = XsMessage_getDataByte(&thisPtr->m_msg, offset+14);
		returnVal->m_day = XsMessage_getDataByte(&thisPtr->m_msg, offset+15);
		returnVal->m_hour = XsMessage_getDataByte(&thisPtr->m_msg, offset+16);
		returnVal->m_min = XsMessage_getDataByte(&thisPtr->m_msg, offset+17);
		returnVal->m_sec = XsMessage_getDataByte(&thisPtr->m_msg, offset+18);
		returnVal->m_valid = XsMessage_getDataByte(&thisPtr->m_msg, offset+19);
	}
	else
	{
		memset(returnVal, 0, sizeof(XsRawGpsTimeUtc));
	}
	return returnVal;
}

/*! \relates XsDataPacket
	\brief Returns 1 if data item contains RawGpsTimeUtc, 0 otherwise
*/
int XsDataPacket_containsRawGpsTimeUtc(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsTimeUtc) != -1);
}

/*! \relates XsDataPacket
	\brief Returns a struct with RawGpsClock data
*/
XsRawGpsSvInfo* XsDataPacket_rawGpsSvInfo(const XsDataPacket* thisPtr, XsRawGpsSvInfo* returnVal)
{
	if (XsDataPacket_containsRawGpsSvInfo(thisPtr))
	{
		XsSize i;
		uint16_t offset = XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSvInfo);

		returnVal->m_itow = XsMessage_getDataLong(&thisPtr->m_msg, offset);
		returnVal->m_nch = XsMessage_getDataByte(&thisPtr->m_msg, offset+4);
		returnVal->m_res1 = XsMessage_getDataByte(&thisPtr->m_msg, offset+5);
		returnVal->m_res2= XsMessage_getDataShort(&thisPtr->m_msg, offset+6);

		offset = offset+8;
		for (i = 0; i < returnVal->m_nch; i++)
		{
			returnVal->m_svInfos[i].m_chn = XsMessage_getDataByte(&thisPtr->m_msg, offset+i*12+0);
			returnVal->m_svInfos[i].m_svid = XsMessage_getDataByte(&thisPtr->m_msg, offset+i*12+1);
			returnVal->m_svInfos[i].m_flags = XsMessage_getDataByte(&thisPtr->m_msg, offset+i*12+2);
			returnVal->m_svInfos[i].m_qi = XsMessage_getDataByte(&thisPtr->m_msg, offset+i*12+3);
			returnVal->m_svInfos[i].m_cno = XsMessage_getDataByte(&thisPtr->m_msg, offset+i*12+4);
			returnVal->m_svInfos[i].m_elev = XsMessage_getDataByte(&thisPtr->m_msg, offset+i*12+5);
			returnVal->m_svInfos[i].m_azim = XsMessage_getDataShort(&thisPtr->m_msg, offset+i*12+6);
			returnVal->m_svInfos[i].m_prres = XsMessage_getDataLong(&thisPtr->m_msg, offset+i*12+8);
		}
	}
	else
	{
		memset(returnVal, 0, sizeof(XsRawGpsSvInfo));
	}
	return returnVal;
}

/*! \brief Returns 1 if data item contains RawGpsSvInfo, 0 otherwise
*/
int XsDataPacket_containsRawGpsSvInfo(const XsDataPacket* thisPtr)
{
	return (XsDataPacket_itemOffsetExact(thisPtr, XDI_GpsSvInfo) != -1);
}

/*! \relates XsDataPacket
	\brief Appends the data items from \a other to the packet

	\details The data items contained in XsDataPacket \a other will be appended to
			this packet. Items that are already contained will not be
			overwritten.

	\param other : The XsDataPacket to read the data items from

	\returns Returns the updated data packet
*/
XsDataPacket* XsDataPacket_append(XsDataPacket* thisPtr, const XsDataPacket* other)
{
	XsSize offset = 0;
	XsSize dataSize;
	XsSize sz;
	XsDataIdentifier id;

	dataSize = XsMessage_dataSize(&other->m_msg);
	while (offset < dataSize)
	{
		id = (XsDataIdentifier) XsMessage_getDataShort(&other->m_msg, offset);
		sz = 3 + XsMessage_getDataByte(&other->m_msg, offset+2);
		if (XsDataPacket_itemOffsetExact(thisPtr, id) == -1)
		{
			// add item
			XsMessage_setDataBuffer(&thisPtr->m_msg, XsMessage_getDataBuffer(&other->m_msg, offset), sz, XsMessage_dataSize(&thisPtr->m_msg));
			++thisPtr->m_itemCount;
		}
		offset += sz;
	}
	return thisPtr;
}

/*! @} */
