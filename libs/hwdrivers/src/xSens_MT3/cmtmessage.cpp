/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*! \file Cmtmessage.cpp

	For information about objects in this file, see the appropriate header:
	\ref Cmtmessage.h

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

#include "cmtmessage.h"

namespace xsens {

#ifdef _LOG_CMT_MSG
#define MSGLOG		CMTLOG
#else
#define MSGLOG(...)
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Create a Message object with the given data length and message Id.
Message::Message(const uint8_t msgId, const uint16_t length, const uint16_t maxLength)
{
	CMT_CHECKASSIGN

	if (maxLength < CMT_MAXMSGLEN)
		m_maxLength = CMT_MAXMSGLEN;
	else
		m_maxLength = maxLength;
	m_buffer = (MessageHeader*) new char[m_maxLength];//(MessageHeader*) malloc(m_maxLength);
	MSGLOG("Message(%02x, %hu, %hu): buffer = %p\n",(int32_t) msgId, length, m_maxLength,m_buffer);
	memset(m_buffer,0,m_maxLength);

	m_buffer->m_preamble = CMT_PREAMBLE;
	m_buffer->m_messageId = msgId;
	m_buffer->m_busId = CMT_BID_MASTER;

	if (length >= 255)
	{
		m_buffer->m_length = CMT_EXTLENCODE;

		m_buffer->m_datlen.m_extended.m_length.m_high = (uint8_t) (length >> 8);
		m_buffer->m_datlen.m_extended.m_length.m_low = (uint8_t) length;

		m_checksum = &(((uint8_t*) m_buffer)[length + CMT_LEN_MSGEXTHEADER]);
		m_checksum[0] = -(msgId + CMT_EXTLENCODE + (uint8_t)length + (length >> 8));
	}
	else
	{
		m_buffer->m_length = (uint8_t) length;
		m_checksum = &(((uint8_t*) m_buffer)[length + CMT_LEN_MSGHEADER]);
		m_checksum[0] = -(msgId + (uint8_t) length);
	}
	m_checksum[0] -= CMT_BID_MASTER;

	m_autoUpdateChecksum = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Create a message from the given source string
Message::Message(const uint8_t* source, const uint16_t size, const uint16_t maxLength)
{
	CMT_CHECKASSIGN

	const MessageHeader* tmp = (MessageHeader*) source;
	uint16_t length;

	if (maxLength < CMT_MAXMSGLEN)
		m_maxLength = CMT_MAXMSGLEN;
	else
		m_maxLength = maxLength;

	if (tmp->m_length == CMT_EXTLENCODE)
		length = ((uint16_t) tmp->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) tmp->m_datlen.m_extended.m_length.m_low) + CMT_LEN_MSGEXTHEADERCS;
	else
		length = tmp->m_length + CMT_LEN_MSGHEADERCS;
	if (size && size < length)
		length = size;

	if ((uint32_t) length > m_maxLength)
		m_maxLength = length;

	m_buffer = (MessageHeader*) new char[m_maxLength];//(MessageHeader*) malloc(m_maxLength);
	MSGLOG("Message(%02x%02x%02x%02x%02x%02x, %hu, %hu): buffer = %p\n",source[0],source[1],source[2],source[3],source[4],source[5], size, m_maxLength,m_buffer);
	if (length < m_maxLength)
		memset(&(((uint8_t*) m_buffer)[length]),0,m_maxLength-length);

	memcpy(m_buffer,source,length);

	m_checksum = &(((uint8_t*) m_buffer)[length-1]);

	m_autoUpdateChecksum = true;
}

Message::Message(const Message& src)
{
	CMT_CHECKASSIGN;

	m_maxLength = src.m_maxLength;
	m_buffer = (MessageHeader*) new char[m_maxLength];
	memcpy(m_buffer,src.m_buffer,m_maxLength);
	ptrdiff_t add = (uint8_t*) src.m_checksum - (uint8_t*) src.m_buffer;
	m_checksum = (uint8_t*) m_buffer + add;
	m_autoUpdateChecksum = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Destroy a Message object.
Message::~Message()
{
	CMT_CHECKASSERT
	MSGLOG("~Message(): buffer = %p\n",m_buffer);
	CHKDELNUL(m_buffer);//free(m_buffer);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Clear all data in the message
void Message::clear(void)
{
	CMT_CHECKASSERT

	memset(m_buffer,0,m_maxLength);
	m_checksum = &(((uint8_t*) m_buffer)[CMT_LEN_MSGHEADER]);
	m_buffer->m_preamble = CMT_PREAMBLE;
	m_buffer->m_busId = CMT_BID_MASTER;
	m_checksum[0] = (uint8_t) (-CMT_BID_MASTER);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the current value of the data as a double (64 bits).
double Message::getDataDouble(const uint16_t offset) const
{
	CMT_CHECKASSERT
	double ret;
	uint8_t* dest = (uint8_t*) &ret;
	uint8_t* src = &(getDataStart()[offset]);
	dest[0] = src[7];	dest[1] = src[6];	dest[2] = src[5];	dest[3] = src[4];
	dest[4] = src[3];	dest[5] = src[2];	dest[6] = src[1];	dest[7] = src[0];
	
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the current value of the data as a float (32 bits).
float Message::getDataFloat(const uint16_t offset) const
{
	CMT_CHECKASSERT
	float ret;
	uint8_t* dest = (uint8_t*) &ret;
	uint8_t* src = &(getDataStart()[offset]);
	dest[0] = src[3];
	dest[1] = src[2];
	dest[2] = src[1];
	dest[3] = src[0];
	
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the current value of the data as a double(64 bits), after converting it from F1220.
double Message::getDataF1220(const uint16_t offset) const
{
	CMT_CHECKASSERT
	double ret;
	int32_t tmp;
	uint8_t* dest = (uint8_t*) &tmp;
	uint8_t* src = &(getDataStart()[offset]);
	dest[0] = src[3];
	dest[1] = src[2];
	dest[2] = src[1];
	dest[3] = src[0];

	ret = ((double) tmp)/1048576.0;
	return ret;
}

// little endian
union Itypes {
	int64_t i64;
	struct {
		int32_t i1,i0;
	} i32;
	struct {
		int16_t s3,s2,s1,s0;
	} i16;
	struct {
		signed char b7,b6,b5,b4,b3,b2,b1,b0;
	} i8;

	double d;
	struct {
		float f1,f0;
	} f32;
};

//////////////////////////////////////////////////////////////////////////////////////////
// Return the current value of the data as a double(64 bits), after converting it from FP1632.
double Message::getDataFP1632(const uint16_t offset) const
{
	CMT_CHECKASSERT

	int16_t fpint;
	int32_t fpfrac;

	uint8_t* dest = (uint8_t*) &fpfrac;
	uint8_t* src = &(getDataStart()[offset]);
	dest[3] = *(src++);
	dest[2] = *(src++);
	dest[1] = *(src++);
	dest[0] = *(src++);

	dest = (uint8_t*) &fpint;
	dest[1] = *(src++);
	dest[0] = *(src++);

	Itypes fp;
	fp.i32.i0 = fpint;
	fp.i32.i1 = fpfrac;

	return (double) fp.i64 / 4294967296.0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return current data value as a double(64 bits), after converting it from
// float, double, FP1632 or FP1220 depending on outputSettings
double Message::getDataFPValue(const uint64_t outputSettings, const uint16_t offset) const
{
	switch (outputSettings & CMT_OUTPUTSETTINGS_DATAFORMAT_MASK)
	{
		case CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT:
			return getDataFloat(offset);

		case CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE:
			return getDataDouble(offset);

		case CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632:
			return getDataFP1632(offset);

		case CMT_OUTPUTSETTINGS_DATAFORMAT_F1220:
			return getDataF1220(offset);
	}
	return 0.0; // should never happen
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return current data values as a double(64 bits), after converting it from
// float, double, FP1632 or FP1220 depending on outputSettings
void Message::getDataFPValue(double *dest, const uint64_t outputSettings, uint16_t offset, const int16_t numValues) const
{
	for (uint16_t i=0; i<numValues; i++) {
		switch (outputSettings & CMT_OUTPUTSETTINGS_DATAFORMAT_MASK)
		{
			case CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT:
				*dest++ = getDataFloat(offset);
				offset += 4;
				break;

			case CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE:
				*dest++ = getDataDouble(offset);
				offset += 8;
				break;

			case CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632:
				*dest++ = getDataFP1632(offset);
				offset += 6;
				break;

			case CMT_OUTPUTSETTINGS_DATAFORMAT_F1220:
				*dest++ = getDataF1220(offset);
				offset += 4;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the current value of the data as an uint32_t (32 bits).
uint32_t Message::getDataLong(const uint16_t offset) const
{
	CMT_CHECKASSERT
	uint32_t ret;
	uint8_t* dest = (uint8_t*) &ret;
	uint8_t* src = &(getDataStart()[offset]);
	dest[0] = src[3];
	dest[1] = src[2];
	dest[2] = src[1];
	dest[3] = src[0];
	
	return ret;
}
	
//////////////////////////////////////////////////////////////////////////////////////////
// Return the current value of the data as an uint16_t (16 bits).
uint16_t Message::getDataShort(const uint16_t offset) const
{
	CMT_CHECKASSERT
	uint16_t ret;
	uint8_t* dest = (uint8_t*) &ret;
	uint8_t* src = &(getDataStart()[offset]);
	dest[0] = src[1];
	dest[1] = src[0];
	
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the length of the message buffer.
uint16_t Message::getDataSize(void) const
{
	CMT_CHECKASSERT
	if (m_buffer->m_length == 255)
		return ((uint16_t) m_buffer->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) m_buffer->m_datlen.m_extended.m_length.m_low);
	else
		return m_buffer->m_length;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Internal function to get the start of the data buffer.
uint8_t* Message::getDataStart(void) const
{
	CMT_CHECKASSERT
	if (m_buffer->m_length == 255)
	{
		return const_cast<uint8_t*>(m_buffer->m_datlen.m_extended.m_data);
	}
	else
		return const_cast<uint8_t*>(m_buffer->m_datlen.m_data);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the length of the message buffer.
uint16_t Message::getTotalMessageSize(void) const
{
	CMT_CHECKASSERT
	if (m_buffer->m_length == 255)
		return ((uint16_t) m_buffer->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) m_buffer->m_datlen.m_extended.m_length.m_low) + CMT_LEN_MSGEXTHEADERCS;
	else
		return m_buffer->m_length + CMT_LEN_MSGHEADERCS;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Message::isChecksumOk(void) const
{
	if (getTotalMessageSize() > m_maxLength)
		return false;
	return calcChecksum() == m_checksum[0];
}

//////////////////////////////////////////////////////////////////////////////////////////
// Read the entire message from the given source string
XsensResultValue Message::loadFromString(const uint8_t* source, const uint16_t size)
{
	CMT_CHECKASSERT
	if (size > m_maxLength)
		return XRV_PARAMINVALID;
	memcpy(m_buffer,source,size);
	m_checksum = ((uint8_t*) m_buffer) + size-1;
	// check the size
	if (m_buffer->m_length == CMT_EXTLENCODE)
	{
		if (((uint16_t) m_buffer->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) m_buffer->m_datlen.m_extended.m_length.m_low) > size-CMT_LEN_MSGEXTHEADERCS)
			return XRV_DATACORRUPT;
	}
	else
	{
		if (m_buffer->m_length > size-CMT_LEN_MSGHEADERCS)
			return XRV_DATACORRUPT;
	}

	if (m_checksum[0] != calcChecksum())
		return XRV_CHECKSUMFAULT;
	return XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::resizeData(const uint16_t newSize)
{
	CMT_CHECKASSERT
	int32_t index, oldLength;

	MSGLOG("Message.resizeData(%hu): buffer = %p\n",newSize,m_buffer);

	if ((uint32_t) (newSize + CMT_LEN_MSGEXTHEADERCS) > m_maxLength)
	{
		uint16_t newLen = (uint16_t) (m_maxLength+m_maxLength);
		if ((newSize + CMT_LEN_MSGEXTHEADERCS) > newLen)
			newLen = newSize + CMT_LEN_MSGEXTHEADERCS;
		m_buffer = (MessageHeader*) realloc(m_buffer,newLen);
		m_maxLength = newLen;
	}

	MSGLOG("Message.resizeData after realloc(%hu): buffer = %p\n",m_maxLength,m_buffer);

	if (newSize >= CMT_EXTLENCODE)
	{
		if (m_buffer->m_length < CMT_EXTLENCODE)
		{
			// make extended, shift all data 2 bytes up
			for (index = (int32_t) m_buffer->m_length; index >= 0; --index)
				m_buffer->m_datlen.m_extended.m_data[index] =
								m_buffer->m_datlen.m_data[index];
	
			oldLength = m_buffer->m_length;
			m_buffer->m_length = CMT_EXTLENCODE;
		}
		else
		{
			oldLength = ((uint16_t) m_buffer->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) m_buffer->m_datlen.m_extended.m_length.m_low);
		}
		m_buffer->m_datlen.m_extended.m_length.m_high = (uint8_t) (newSize >> 8);
		m_buffer->m_datlen.m_extended.m_length.m_low = (uint8_t) newSize;

		for (index = oldLength; index < newSize; ++index)
			m_buffer->m_datlen.m_extended.m_data[index] = 0;

		m_checksum = &m_buffer->m_datlen.m_extended.m_data[newSize];
	}
	else
	{
		if (m_buffer->m_length == CMT_EXTLENCODE)
		{
			oldLength = ((uint16_t) m_buffer->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) m_buffer->m_datlen.m_extended.m_length.m_low);
			// un-extend, shift all data 2 bytes down
			for (index = 0; index < newSize; ++index)
				m_buffer->m_datlen.m_data[index] =
								m_buffer->m_datlen.m_extended.m_data[index];
		}
		else
			oldLength = m_buffer->m_length;
		m_buffer->m_length = (uint8_t) newSize;
		for (index = oldLength; index < newSize; ++index)
			m_buffer->m_datlen.m_data[index] = 0;
		m_checksum = &m_buffer->m_datlen.m_data[newSize];
	}
	if (m_autoUpdateChecksum)
		recomputeChecksum();
	MSGLOG("Message.resizeData end(%hu): buffer = %p\n",m_maxLength,m_buffer);
	CMT_CHECKASSERT
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the new value of the m_busId field and update the checksum.
void Message::setBusId(const uint8_t busId)
{
	CMT_CHECKASSERT
	if (m_autoUpdateChecksum)
		m_checksum[0] += m_buffer->m_busId - busId;
	m_buffer->m_busId = busId;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Write a string of bytes into the data buffer.
void Message::setDataBuffer(const uint8_t* data, const uint16_t offset,
						const uint16_t count)
{
	CMT_CHECKASSERT
	if (getDataSize() < offset+count)
		resizeData(offset+count);

	if (count > 0)
	{
		uint8_t * dest = &(getDataStart()[offset]);
		for (uint16_t i = 0;i < count;++i)
		{
			if (m_autoUpdateChecksum)
				m_checksum[0] += dest[i] - data[i];
			dest[i] = data[i];
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::setDataByte(const uint8_t data, const uint16_t offset)
{
	CMT_CHECKASSERT
	if (getDataSize() < offset + 1)
		resizeData(offset+1);

	uint8_t* dest = &(getDataStart()[offset]);
	if (m_autoUpdateChecksum)
		m_checksum[0] += dest[0] - data;
	dest[0]	= data;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::setDataDouble(const double data, const uint16_t offset)
{
	CMT_CHECKASSERT
	if (getDataSize() < offset+8)
		resizeData(offset+8);

	uint8_t* dest = &(getDataStart()[offset]);
	const uint8_t* cdata = (const uint8_t*) &data;
	if (m_autoUpdateChecksum)
		m_checksum[0] += dest[0]  + dest[1]  + dest[2]  + dest[3]
					  +	 dest[4]  + dest[5]  + dest[6]  + dest[7]
					  -  cdata[0] - cdata[1] - cdata[2] - cdata[3]
					  -  cdata[4] - cdata[5] - cdata[6] - cdata[7];

	const uint8_t* src = (const uint8_t*) &data;
	dest[0] = src[7];	dest[1] = src[6];	dest[2] = src[5];	dest[3] = src[4];
	dest[4] = src[3];	dest[5] = src[2];	dest[6] = src[1];	dest[7] = src[0];
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::setDataFloat(const float data, const uint16_t offset)
{
	CMT_CHECKASSERT
	if (getDataSize() < offset+4)
		resizeData(offset+4);

	uint8_t* dest = &(getDataStart()[offset]);
	const uint8_t* cdata = (const uint8_t*) &data;
	if (m_autoUpdateChecksum)
		m_checksum[0] += dest[0]  + dest[1]  + dest[2]  + dest[3]
					  -  cdata[0] - cdata[1] - cdata[2] - cdata[3];

	const uint8_t* src = (const uint8_t*) &data;
	dest[0] = src[3];
	dest[1] = src[2];
	dest[2] = src[1];
	dest[3] = src[0];
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::setDataF1220(const double data, const uint16_t offset)
{
	int32_t val = (int32_t) (data*1048576.0);
	setDataLong(val,offset);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the current value of the data as a double(64 bits), after converting it from F1220.
void Message::setDataFP1632(const double data, const uint16_t offset)
{
	CMT_CHECKASSERT

	if (getDataSize() < offset+6)
		resizeData(offset+6);

	Itypes fp;
	fp.d = data;
	int32_t dexp = ((fp.i32.i0 & (0x7fffffffL)) >> 20) - 1023;

	int16_t fpint;
	int32_t fpfrac;

	if (dexp <= 14)
	{
		fp.i16.s0 = (fp.i16.s0 & 0x000F) | 0x0010;
		if (data < 0)
			fp.i64 = -fp.i64;
		if (dexp > -32)
			fp.i64 = fp.i64 >> (20-dexp);	// 52-32 - exponent
		else
			fp.i64 = fp.i64 >> 52;	// this could be optimized?
		fpint = fp.i16.s1;
		fpfrac = fp.i32.i1;
	}
	else
	{
		if (data < 0)
		{
			fpint = ((int16_t) (0x8000));
			fpfrac = 0;
		}
		else
		{
			fpint = 0x7fff;
			fpfrac = -1;
		}
	}

	uint8_t* src = (uint8_t*) &fpfrac;
	uint8_t* dest = &(getDataStart()[offset]);

	*(dest++) = src[3];
	*(dest++) = src[2];
	*(dest++) = src[1];
	*(dest++) = src[0];

	src = (uint8_t*) &fpint;
	*(dest++) = src[1];
	*(dest++) = src[0];
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::setDataFPValue(const uint64_t outputSettings, const double data, const uint16_t offset)
{
	switch (outputSettings & CMT_OUTPUTSETTINGS_DATAFORMAT_MASK)
	{
		case CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT:
			return setDataFloat((const float)data, offset);

		case CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE:
			return setDataDouble(data, offset);

		case CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632:
			return setDataFP1632(data, offset);

		case CMT_OUTPUTSETTINGS_DATAFORMAT_F1220:
			return setDataF1220(data, offset);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::setDataFPValue(const uint64_t outputSettings, const double *data, uint16_t offset, const uint16_t numValues)
{
	for (uint16_t i=0; i<numValues; i++) {
		switch (outputSettings & CMT_OUTPUTSETTINGS_DATAFORMAT_MASK)
		{
			case CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT:
				setDataFloat((float)data[i], offset);
				offset += 4;
				break;

			case CMT_OUTPUTSETTINGS_DATAFORMAT_DOUBLE:
				setDataDouble(data[i], offset);
				offset += 8;
				break;

			case CMT_OUTPUTSETTINGS_DATAFORMAT_FP1632:
				setDataFP1632(data[i], offset);
				offset += 6;
				break;

			case CMT_OUTPUTSETTINGS_DATAFORMAT_F1220:
				setDataF1220(data[i], offset);
				offset += 4;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::setDataLong(const uint32_t data, const uint16_t offset)
{
	CMT_CHECKASSERT
	if (getDataSize() < offset+4)
		resizeData(offset+4);

	uint8_t* dest = &(getDataStart()[offset]);
	if (m_autoUpdateChecksum)
		m_checksum[0] +=(uint8_t)(dest[0] + dest[1] + dest[2] + dest[3] - (data & 0xFF)-
						((data >> 8) & 0xFF) - ((data >> 16) & 0xFF) - ((data >> 24) & 0xFF));

	const uint8_t* src = (const uint8_t*) &data;
	dest[0] = src[3];
	dest[1] = src[2];
	dest[2] = src[1];
	dest[3] = src[0];
}

//////////////////////////////////////////////////////////////////////////////////////////
void Message::setDataShort(const uint16_t data, const uint16_t offset)
{
	CMT_CHECKASSERT
	if (getDataSize() < offset+2)
		resizeData(offset+2);

	uint8_t* dest = &(getDataStart()[offset]);
	if (m_autoUpdateChecksum)
		m_checksum[0] += dest[0] + dest[1] - (data & 255) - (data >> 8);

	const uint8_t* src = (const uint8_t*) &data;
	dest[0] = src[1];
	dest[1] = src[0];
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the new value of the m_messageId field and update the checksum.
void Message::setMessageId(const uint8_t msgId)
{
	CMT_CHECKASSERT
	if (m_autoUpdateChecksum)
		m_checksum[0] += m_buffer->m_messageId - msgId;
	m_buffer->m_messageId =msgId;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Copy message src into this
void Message::operator = (const Message& src)
{
	CMT_CHECKASSERT

	if (m_maxLength != src.m_maxLength)
	{
		CHKDELNUL(m_buffer);
		m_maxLength = src.m_maxLength;
		m_buffer = (MessageHeader*) new char[m_maxLength];
	}
	memcpy(m_buffer,src.m_buffer,m_maxLength);
	ptrdiff_t add = (uint8_t*) src.m_checksum - (uint8_t*) src.m_buffer;
	m_checksum = (uint8_t*) m_buffer + add;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Remove a number of bytes from the message (this will reduce the message size)
void Message::deleteData(uint16_t size, uint16_t offset)
{
	if (size == 0)
		return;
	uint16_t oldSize = getDataSize();
	uint16_t newSize;
	if (offset >= oldSize)
		return;

	if (oldSize > offset + size)
		newSize = oldSize - size;
	else
		newSize = offset;

	// shift all bytes after the offset down by size positions
	uint8_t* address = getDataBuffer();
	for (uint16_t i = offset; i < newSize; ++i)
		address[i] = address[i+size];

	// reduce the message length
	resizeData(newSize);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Insert a number of bytes into the message (this will increase the message size)
void Message::insertData(uint16_t size, uint16_t offset)
{
	if (size == 0)
		return;
	// stretch the message
	uint16_t oldSize = getDataSize();
	resizeData(oldSize + size);
	// shift the bytes after the offset up by size positions
	uint8_t* address = getDataBuffer();
	for (uint16_t i = oldSize-1; i >= offset && i < oldSize; --i)
		address[i+size] = address[i];
}

//////////////////////////////////////////////////////////////////////////////////////////
// Compute the checksum of the given byte string.
uint8_t computeChecksum(const uint8_t* buffer, uint32_t length)
{
	uint8_t cs = 0;
	while (length--)
		cs -= *(buffer++);

	return cs;
}

} // end of xsens namespace
