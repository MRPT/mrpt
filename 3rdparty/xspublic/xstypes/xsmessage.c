
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

#include "xsmessage.h"
#include <stdlib.h>
#include <memory.h>		// memset
#include "xsbusid.h"
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////////////////
// Field message indices
#define XS_IND_PREAMBLE				0
#define XS_IND_BID						1
#define XS_IND_MID						2
#define XS_IND_LEN						3
#define XS_IND_DATA0					4
#define XS_IND_LENEXTH					4
#define XS_IND_LENEXTL					5
#define XS_IND_DATAEXT0				6

#define XS_SELFTEST_OK					0x1FF

// message data lengths
#define XS_LEN_TRANSPORTMODE			1
#define XS_LEN_DEVICEID				4
#define XS_LEN_INITBUSRESULTS			4
#define XS_LEN_PERIOD					2
#define XS_LEN_BUSPWR					2
#define XS_LEN_DATALENGTH				2
#define XS_LEN_CONFIGURATION			118
#define XS_LEN_FIRMWAREREV				3
#define XS_LEN_BTDISABLE				1
#define XS_LEN_OPMODE					1
#define XS_LEN_BAUDRATE				1
#define XS_LEN_SYNCMODE				1
#define XS_LEN_PRODUCTCODE				20
#define XS_LEN_PROCESSINGFLAGS			2
#define XS_LEN_XMPWROFF				0
#define XS_LEN_OUTPUTMODE		 		2
#define XS_LEN_OUTPUTSETTINGS		 	4
#define XS_LEN_OUTPUTSKIPFACTOR		2
#define XS_LEN_SYNCINMODE				2
#define XS_LEN_SYNCINSKIPFACTOR		2
#define XS_LEN_SYNCINOFFSET			4
#define XS_LEN_SYNCOUTMODE				2
#define XS_LEN_SYNCOUTSKIPFACTOR		2
#define XS_LEN_SYNCOUTOFFSET			4
#define XS_LEN_SYNCOUTPULSEWIDTH		4
#define XS_LEN_ERRORMODE				2
#define XS_LEN_TRANSMITDELAY			2
#define XS_LEN_OBJECTALIGNMENT			36
#define XS_LEN_XMERRORMODE				2
#define XS_LEN_BUFFERSIZE				2
#define XS_LEN_HEADING		 			4
#define XS_LEN_MAGNETICFIELD		 	12
#define XS_LEN_LOCATIONID				2
#define XS_LEN_EXTOUTPUTMODE			2
#define XS_LEN_INITTRACKMODE			2
#define XS_LEN_STOREFILTERSTATE			0
#define XS_LEN_UTCTIME					12
#define XS_LEN_FILTERPROFILELABEL			20
#define XS_LEN_FILTERPROFILEFULL			(1+1+XS_LEN_FILTERPROFILELABEL)
#define XS_LEN_AVAILABLEFILTERPROFILES		(XS_MAX_FILTERPROFILES_IN_MT*XS_LEN_FILTERPROFILEFULL)
#define XS_LEN_REQFILTERPROFILEACK			2
#define XS_LEN_SETFILTERPROFILE				2
#define XS_LEN_GRAVITYMAGNITUDE		4
#define XS_LEN_GNSSLEVERARM				12
#define XS_LEN_LATLONALT				18
#define XS_LEN_SETNOROTATION			2
#define XS_LEN_FILTERSETTINGS			4
#define XS_LEN_AMD						2
#define XS_LEN_RESETORIENTATION		2
#define XS_LEN_GNSSSTATUS				(1+5*16)
#define XS_LEN_CLIENTUSAGE				1
#define XS_LEN_CLIENTPRIORITY			1
#define XS_LEN_WIRELESSCONFIG			6
#define XS_LEN_INFOREQUEST				1
#define XS_LEN_SETOUTPUTTRIGGER		10
#define XS_LEN_SETINPUTTRIGGER			10

// MTData defines
// Length of data blocks in bytes
#define XS_LEN_RAWDATA					20
#define XS_LEN_CALIBDATA				36
#define XS_LEN_CALIB_ACCDATA			12
#define XS_LEN_CALIB_GYRDATA			12
#define XS_LEN_CALIB_MAGDATA			12
#define XS_LEN_ORIENT_QUATDATA			16
#define XS_LEN_ORIENT_EULERDATA		12
#define XS_LEN_ORIENT_MATRIXSTA		36
#define XS_LEN_SAMPLECNT				2
#define XS_LEN_TEMPDATA				4

// Length of data blocks in floats
#define XS_LEN_CALIBDATA_FLT			9
#define XS_LEN_TEMPDATA_FLT			1
#define XS_LEN_ORIENT_QUATDATA_FLT		4
#define XS_LEN_ORIENT_EULERDATA_FLT	3
#define XS_LEN_ORIENT_MATRIXSTA_FLT	9

#pragma pack(push, 1)
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
#pragma pack(pop)


/*! \addtogroup cinterface C Interface
	@{
*/

typedef union Itypes Itypes;

/*! \brief Calculate the sum of the values in the buffer
	\details This function calculates the sum of the byte values for the first \a count bytes in the \a buffer
	\param buffer An array of (unsigned) bytes
	\param count The number of bytes in the buffer
	\returns The unsigned sum of the byte values in the buffer modulo 256
*/
uint8_t byteSum(const uint8_t* buffer, XsSize count)
{
	register uint8_t sum = 0;
	for (; count; --count, sum += *(buffer++));
	return sum;
}

/*! \brief Get the buffer at offset \a offset */
static inline uint8_t *XsMessage_dataAtOffset(XsMessage *thisPtr, XsSize offset)
{
	XsMessageHeader* hdr;

	assert(thisPtr->m_message.m_data);
	assert(offset < XsMessage_dataSize(thisPtr));

	hdr = (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
	if (hdr->m_length != 255)
		return hdr->m_datlen.m_data + offset;
	return hdr->m_datlen.m_extended.m_data + offset;
}

/*! \brief return the const data at offset \a offset */
static inline const uint8_t *XsMessage_cdataAtOffset(XsMessage const *thisPtr, XsSize offset)
{
	return XsMessage_dataAtOffset((XsMessage*)thisPtr, offset);
}

/*! \brief Make sure the data buffer is large enough to hold a new data item of \a sizeofValue */
static inline void XsMessage_ensureDataSize(XsMessage *thisPtr, XsSize offset, XsSize sizeofValue)
{
	if (XsMessage_dataSize(thisPtr) < offset + sizeofValue)
		XsMessage_resizeData(thisPtr, offset + sizeofValue);
}

/*! \brief Update the message checksum with the passed value */
static inline void XsMessage_updateChecksumWithValue(XsMessage *thisPtr, const void *value, XsSize sizeofValue, XsSize offset)
{
	if (thisPtr->m_autoUpdateChecksum)
	{
		thisPtr->m_checksum[0] += byteSum(XsMessage_getDataBuffer(thisPtr, offset), sizeofValue);
		thisPtr->m_checksum[0] -= byteSum(value, sizeofValue);
	}
}

/*! \brief Swap the endianness based on the data size */
static inline void swapEndian(void *data, const XsSize size)
{
	uint16_t *i16;
	uint32_t *i32;
	uint64_t *i64;

	switch (size)
	{
	case sizeof(char):
		break;
	case sizeof(*i16):
		i16 = data;
		*i16 = swapEndian16(*i16);
		break;
	case sizeof(*i32):
		i32 = data;
		*i32 = swapEndian32(*i32);
		break;
	case sizeof(*i64):
		i64 = data;
		*i64 = swapEndian64(*i64);
		break;
	default:
		assert(0);
	}
}

/*! \brief Get data of size \a size at \a offset, and put it byteswapped into \a value */
void XsMessage_getEndianCorrectData(XsMessage const* thisPtr, void *value, XsSize size, XsSize offset)
{
	memcpy(value, (void const*) XsMessage_cdataAtOffset(thisPtr, offset), size);
	swapEndian(value, size);
}

/*! \brief Set value \a value of size \a size byteswapped at \a offset */
void XsMessage_setEndianCorrectData(XsMessage *thisPtr, void const *value, XsSize size, XsSize offset)
{
	void* dest;
	XsMessage_ensureDataSize(thisPtr, offset, size);
	XsMessage_updateChecksumWithValue(thisPtr, value, size, offset);
	dest = XsMessage_dataAtOffset(thisPtr, offset);
	memcpy(dest, value, size);
	swapEndian(dest, size);
}

/*! \brief This function initializes the %XsMessage object and reserves \a dataSize bytes for data

	\param[in] dataSize the expected size of the message payload
*/
void XsMessage_constructSized(XsMessage* thisPtr, XsSize dataSize)
{
	XsSize msgSize;
	XsMessageHeader* hdr;

	if (dataSize < 255)
		msgSize = dataSize + 5;
	else
		msgSize = dataSize + 7;

	XsByteArray_construct(&thisPtr->m_message, msgSize, 0);
	memset(thisPtr->m_message.m_data, 0, msgSize);
	hdr = (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
	hdr->m_preamble = XS_PREAMBLE;
	hdr->m_messageId = 0;
	hdr->m_busId = XS_BID_MASTER;

	if (dataSize < XS_EXTLENCODE)
	{
		hdr->m_length = (uint8_t) dataSize;
		*((uint8_t**) &thisPtr->m_checksum) = &hdr->m_datlen.m_data[dataSize];
		thisPtr->m_checksum[0] = (uint8_t)-(int8_t)(uint8_t)dataSize;
	}
	else
	{
		hdr->m_length = XS_EXTLENCODE;
		hdr->m_datlen.m_extended.m_length.m_high = (uint8_t) (dataSize >> 8);
		hdr->m_datlen.m_extended.m_length.m_low = (uint8_t) dataSize;
		*((uint8_t**) &thisPtr->m_checksum) = &hdr->m_datlen.m_extended.m_data[dataSize];
		thisPtr->m_checksum[0] = (uint8_t) -(hdr->m_datlen.m_extended.m_length.m_high + hdr->m_datlen.m_extended.m_length.m_low + XS_EXTLENCODE);
	}
	thisPtr->m_checksum[0] -= hdr->m_busId;
}

/*! \brief This function initializes the %XsMessage object
*/
void XsMessage_construct(XsMessage* thisPtr)
{
	XsMessage_constructSized(thisPtr, 0);
}

/*! \brief Construct an XsMessage as a copy of XsMessage \a src
*/
void XsMessage_copyConstruct(XsMessage* thisPtr, XsMessage const* src)
{
	if (!src)
		XsMessage_construct(thisPtr);
	else
	{
		XsMessageHeader* hdr;
		XsSize dataSize;
		XsArray_copyConstruct(&thisPtr->m_message, &src->m_message);
		thisPtr->m_autoUpdateChecksum = src->m_autoUpdateChecksum;
		hdr = (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
		dataSize = XsMessage_dataSize(thisPtr);
		if (dataSize >= 255)
			*((uint8_t**) &thisPtr->m_checksum) = &hdr->m_datlen.m_extended.m_data[dataSize];
		else
			*((uint8_t**) &thisPtr->m_checksum) = &hdr->m_datlen.m_data[dataSize];
	}
}

/*! \brief This function reinitializes the %XsMessage object and reserves \a dataSize bytes for data
	\param[in] dataSize the expected size of the message payload
*/
void XsMessage_assign(XsMessage* thisPtr, XsSize dataSize)
{
	XsMessage_destruct(thisPtr);
	XsMessage_constructSized(thisPtr, dataSize);
}

/*! \brief This function initializes the %XsMessage object and reserves \a msgSize bytes for data, it then copies in the data from \a src
	\param msgSize the size of the data pointed to by src
	\param src the data to load the message from
	\note This is a constructor! Previous contents will be overwritten without first freeing them!
*/
void XsMessage_load(XsMessage* thisPtr, XsSize msgSize, unsigned char const* src)
{
	XsByteArray_construct(&thisPtr->m_message, msgSize, src);
	*((uint8_t**) &thisPtr->m_checksum) = &thisPtr->m_message.m_data[XsMessage_getTotalMessageSize(thisPtr)-1];
}

/*! \brief This function clears the data in the message

*/
void XsMessage_destruct(XsMessage* thisPtr)
{
	XsArray_destruct(&thisPtr->m_message);
	*((uint8_t**) &thisPtr->m_checksum) = 0;
}

/*! \brief This function copies from \a thisPtr to \a copy
	\param copy the object to copy to
*/
void XsMessage_copy(XsMessage* copy, XsMessage const* thisPtr)
{
	XsArray_copy(&copy->m_message, &thisPtr->m_message);
	*((uint8_t**) &copy->m_checksum) = &copy->m_message.m_data[XsMessage_getTotalMessageSize(copy)-1];
	copy->m_autoUpdateChecksum = thisPtr->m_autoUpdateChecksum;
}

/*! \brief This function returns the datasize of the message in \a thisptr
	\returns the size of the message payload
*/
XsSize XsMessage_dataSize(XsMessage const* thisPtr)
{
	XsMessageHeader* hdr;

	if (!thisPtr->m_message.m_data)
		return 0;

	hdr = (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
	if (hdr->m_length == 255)
		return (((XsSize) hdr->m_datlen.m_extended.m_length.m_high) << 8) + hdr->m_datlen.m_extended.m_length.m_low;
	else
		return ((XsSize) hdr->m_length);
}

/*! \brief This function returns a const pointer to the \a offset in the data of the message in \a thisptr

	\param offset the offset of the data to be returned

	\returns a pointer to the data at offset \a offset
*/
const uint8_t* XsMessage_constData(XsMessage const* thisPtr, XsSize offset)
{
	if (!thisPtr->m_message.m_data)
		return 0;
	return XsMessage_cdataAtOffset(thisPtr, offset);
}

/*! \brief This function returns a const pointer to the header of the message in \a thisptr


	\returns a pointer to the start of the message
*/
const uint8_t* XsMessage_getMessageStart(XsMessage const* thisPtr)
{
	return thisPtr->m_message.m_data;
}

/*! \brief Return the length of the message buffer.

	The function returns the total size of the message, including the checksum. This
	is in effect the number of bytes that would be transferred if the message were to
	be sent over a communications channel.

	\returns the total message size
*/
XsSize XsMessage_getTotalMessageSize(XsMessage const* thisPtr)
{
	XsMessageHeader* hdr;

	if (!thisPtr->m_message.m_data)
		return 0;

	hdr = (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
	if (hdr->m_length == 255)
		return (((XsSize) hdr->m_datlen.m_extended.m_length.m_high) << 8) + hdr->m_datlen.m_extended.m_length.m_low + 7;
	else
		return ((XsSize) hdr->m_length) + 5;
}

/*! \brief Returns the byte value at \a offset in the data of the message

	\param offset the offset in the payload at which to read data

	\returns the byte at offset \a offset in the message payload
*/
uint8_t XsMessage_getDataByte(XsMessage const* thisPtr, XsSize offset)
{
	return *XsMessage_cdataAtOffset(thisPtr, offset);
}

/*! \brief Returns the short value at \a offset in the data of the message

	\param offset the offset in the payload at which to read data

	\returns the 16-bit integer value at offset \a offset in the message payload
*/
uint16_t XsMessage_getDataShort(XsMessage const* thisPtr, XsSize offset)
{
	uint16_t ret;
	XsMessage_getEndianCorrectData(thisPtr, &ret, sizeof(ret), offset);
	return ret;
}

/*! \brief Returns the long value at \a offset in the data of the message

	\param offset the offset in the payload at which to read data

	\returns the 32-bit integer value at offset \a offset in the message payload
*/
uint32_t XsMessage_getDataLong(XsMessage const* thisPtr, XsSize offset)
{
	uint32_t ret;
	XsMessage_getEndianCorrectData(thisPtr, &ret, sizeof(ret), offset);
	return ret;
}

/*! \brief Returns the long value at \a offset in the data of the message

	\param offset the offset in the payload at which to read data

	\returns the 64-bit integer value at offset \a offset in the message payload
*/
uint64_t XsMessage_getDataLongLong(XsMessage const* thisPtr, XsSize offset)
{
	uint64_t ret;
	XsMessage_getEndianCorrectData(thisPtr, &ret, sizeof(ret), offset);
	return ret;
}

/*! \brief Returns the float value at \a offset in the data of the message

	\param offset the offset in the payload at which to read data

	\returns the single precision float value at offset \a offset in the message payload
*/
float XsMessage_getDataFloat(XsMessage const* thisPtr, XsSize offset)
{
	float ret;
	XsMessage_getEndianCorrectData(thisPtr, &ret, sizeof(ret), offset);
	return ret;
}

/*! \brief Returns the double at \a offset in the data of the message
	\param offset the offset in the payload at which to read data

	\returns the double precision floating point value at offset \a offset in the message payload
*/
double XsMessage_getDataDouble(XsMessage const* thisPtr, XsSize offset)
{
	double ret;
	XsMessage_getEndianCorrectData(thisPtr, &ret, sizeof(ret), offset);
	return ret;
}

/*! \brief Returns the F12.20 value at \a offset in the data of the message
	\param offset the offset in the payload at which to read data

	\returns the 12.20 fixed point value at offset \a offset in the message payload
*/
double XsMessage_getDataF1220(XsMessage const* thisPtr, XsSize offset)
{
	int32_t tmp;
	Itypes rv;
	tmp = (int32_t) XsMessage_getDataLong(thisPtr, offset);

	rv.d = ((double) tmp)/1048576.0;
	rv.i64 = (rv.i64 & ~1LL) | (tmp & 1);
	return rv.d;
}

/*! \brief Returns the F16.32 value at \a offset in the data of the message
	\param offset the offset in the payload at which to read data

	\returns the 16.32 fixed point value at offset \a offset in the message payload
*/
double XsMessage_getDataFP1632(XsMessage const* thisPtr, XsSize offset)
{
	int16_t fpint;
	int32_t fpfrac;
	Itypes fp, rv;

	fpfrac = (int32_t) XsMessage_getDataLong(thisPtr, offset);
	fpint = (int16_t) XsMessage_getDataShort(thisPtr, offset+4);

	fp.i32.i0 = fpint;
	fp.i32.i1 = fpfrac;

	rv.d = (double) fp.i64 / 4294967296.0;
	rv.i64 = (rv.i64 & ~1LL) | (fpfrac & 1);
	return rv.d;
}

/*! \brief Returns a const pointer to the data buffer of the message

	\param offset the offset in the payload at which to read data


	\returns a const pointer to the data buffer of the message
*/
const uint8_t* XsMessage_getDataBuffer(XsMessage const* thisPtr, XsSize offset)
{
	return XsMessage_constData(thisPtr, offset);
}

/*! \brief Set the byte at \a offset in the message to \a value

	\param value the 8-bit value to set
	\param offset the offset in the message payload at which to write the data
*/
void XsMessage_setDataByte(XsMessage* thisPtr, uint8_t value, XsSize offset)
{
	XsMessage_setEndianCorrectData(thisPtr, &value, sizeof(value), offset);
}

/*! \brief Sets the short at \a offset in the message to \a value

	\param value the 16-bit value to set
	\param offset the offset in the message payload at which to write the data
*/
void XsMessage_setDataShort(XsMessage* thisPtr, uint16_t value, XsSize offset)
{
	XsMessage_setEndianCorrectData(thisPtr, &value, sizeof(value), offset);
}

/*! \brief Sets the long at \a offset in the message to \a value

	\param value the 32-bit value to set
	\param offset the offset in the message payload at which to write the data
*/
void XsMessage_setDataLong(XsMessage* thisPtr, uint32_t value, XsSize offset)
{
	XsMessage_setEndianCorrectData(thisPtr, &value, sizeof(value), offset);
}

/*! \brief Sets the long at \a offset in the message to \a value

	\param value the 64-bit value to set
	\param offset the offset in the message payload at which to write the data
*/
void XsMessage_setDataLongLong(XsMessage* thisPtr, uint64_t value, XsSize offset)
{
	XsMessage_setEndianCorrectData(thisPtr, &value, sizeof(value), offset);
}


/*! \brief Sets the float at \a offset in the message to \a value

	\param value the single precision floating point value to set
	\param offset the offset in the message payload at which to write the data
*/
void XsMessage_setDataFloat(XsMessage* thisPtr, float value, XsSize offset)
{
	XsMessage_setEndianCorrectData(thisPtr, &value, sizeof(value), offset);
}

/*! \brief Sets the double at \a offset in the message to \a value

	\param value the double precision floating point value to set
	\param offset the offset in the message payload at which to write the data
*/
void XsMessage_setDataDouble(XsMessage* thisPtr, double value, XsSize offset)
{
	XsMessage_setEndianCorrectData(thisPtr, &value, sizeof(value), offset);
}

/*! \brief Sets the F12.20 at \a offset in the message to \a value

	\param value the 12.20 fixed point value to set
	\param offset the offset in the message payload at which to write the data
*/
void XsMessage_setDataF1220(XsMessage* thisPtr, double value, XsSize offset)
{
	Itypes fp;
	uint32_t val;

	fp.d = value;
	val = (uint32_t) (int32_t) (value*1048576.0);

	XsMessage_setDataLong(thisPtr, (val & ~1UL) | (fp.i64 & 1), offset);
}

/*! \brief Sets the F16.32 at \a offset in the message to \a value

	\param value the 16.32 fixed point value to set
	\param offset the offset in the message payload at which to write the data
*/
void XsMessage_setDataFP1632(XsMessage* thisPtr, double value, XsSize offset)
{
	Itypes fp;
	int16_t fpint;
	int32_t fpfrac;
	int32_t dexp;
	uint32_t b;

	fp.d = value;
	b = (uint32_t) (fp.i64 & 1);
	dexp = ((fp.i32.i0 & (0x7fffffffL)) >> 20) - 1023;

	if (dexp <= 14)
	{
		fp.i16.s0 = (fp.i16.s0 & 0x000F) | 0x0010;
		if (value < 0)
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
		if (value < 0)
		{
			fpint = ((int16_t) (uint16_t) (0x8000));
			fpfrac = 0;
		}
		else
		{
			fpint = 0x7fff;
			fpfrac = -1;
		}
	}

	XsMessage_setDataLong(thisPtr, (fpfrac & ~1L) | b, offset);
	XsMessage_setDataShort(thisPtr, (uint16_t) fpint, offset+(XsSize)4);
}

/*! \brief Puts \a size number of bytes from \a buffer into the message at \a offset
	\remarks The buffersize of will be increased if \a buffer is too large

	\param buffer the buffer to copy
	\param size the size of the buffer
	\param offset the offset at which to copy the data
*/
void XsMessage_setDataBuffer(XsMessage* thisPtr, const uint8_t* buffer, XsSize size, XsSize offset)
{
	XsMessage_ensureDataSize(thisPtr, offset, size);
	XsMessage_updateChecksumWithValue(thisPtr, buffer, size, offset);
	memcpy(XsMessage_dataAtOffset(thisPtr, offset), buffer, size);
}

/*!	\relates XsMessage
	\brief Returns the byte size of \a id if the format is a floating point format

	\param id : The XsDataIdentifier to query
	\returns Returns the byte size of XsDataIdentifier \a id
*/
uint8_t XsMessage_getFPValueSize(XsDataIdentifier id)
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

/*! \returns a double converted from float \a f

	\param[in] f The float to convert
*/
static double convertFromFloat(float f)
{
	Itypes rv, tmp;
	tmp.f32.f0 = f;
	rv.d = (double) f;
	rv.i64 = (rv.i64 & ~1LL) | (tmp.i32.i0 & 1);
	return rv.d;
}

/*! \returns a float converted from double \a d

	\param[in] d The double to cenvert
*/
static float convertToFloat(double d)
{
	Itypes rv, tmp;
	tmp.d = d;
	rv.f32.f0 = (float) d;
	rv.i32.i0 = (rv.i32.i0 & ~1L) | (tmp.i64 & 1);
	return rv.f32.f0;
}

/*! \brief Return current data values as double, conversion depends on outputSetting.

	\param dest destination array
	\param dataIdentifier Data identifier containing data precision
	\param offset offset in the data buffer from where to start reading.
	\param numValues number of values to be read
*/
void XsMessage_getDataFPValuesById(XsMessage const* thisPtr, XsDataIdentifier dataIdentifier, double *dest, XsSize offset, XsSize numValues)
{
	XsSize i;
	for (i=0; i<numValues; i++)
	{
		switch (dataIdentifier & XDI_SubFormatMask)
		{
		case XDI_SubFormatFloat:
			*dest++ = convertFromFloat(XsMessage_getDataFloat(thisPtr, offset));
			offset += 4;
			break;

		case XDI_SubFormatDouble:
			*dest++ = XsMessage_getDataDouble(thisPtr, offset);
			offset += 8;
			break;

		case XDI_SubFormatFp1632:
			*dest++ = XsMessage_getDataFP1632(thisPtr, offset);
			offset += 6;
			break;

		case XDI_SubFormatFp1220:
			*dest++ = XsMessage_getDataF1220(thisPtr, offset);
			offset += 4;
			break;

		default:
			*dest++ = 0;
			break;
		}
	}
}

/*! \brief Write a number of floating/fixed point values into to the data buffer, conversion depends on outputSettings

	\param dataIdentifier Data Identifier
	\param data		The data array to be written to the buffer.
	\param offset Offset in the data buffer from where to start writing.
	\param numValues number of values to be written
*/
void XsMessage_setDataFPValuesById(XsMessage* thisPtr, XsDataIdentifier dataIdentifier, double const*data, XsSize offset, XsSize numValues)
{
	XsSize i;
	for (i=0; i<numValues; i++)
	{
		switch (dataIdentifier & XDI_SubFormatMask)
		{
		case XDI_SubFormatFloat:
			XsMessage_setDataFloat(thisPtr, convertToFloat(data[i]), offset);
			offset += 4;
			break;

		case XDI_SubFormatDouble:
			XsMessage_setDataDouble(thisPtr, data[i], offset);
			offset += 8;
			break;

		case XDI_SubFormatFp1632:
			XsMessage_setDataFP1632(thisPtr, data[i], offset);
			offset += 6;
			break;

		case XDI_SubFormatFp1220:
			XsMessage_setDataF1220(thisPtr, data[i], offset);
			offset += 4;
			break;

		default:
			break;
		}
	}
}

/*! \brief Return current data values as XsReal, conversion is done automatically based on data identifier.

	This function behaves exactly like XsMessage_getDataFPValuesById, with the exception that it expects an XsReal array.

	\param dest destination array
	\param dataIdentifier Data identifier containing data precision
	\param offset offset in the data buffer from where to start reading.
	\param numValues number of values to be read
*/
void XsMessage_getDataRealValuesById(XsMessage const* thisPtr, XsDataIdentifier dataIdentifier, XsReal *dest, XsSize offset, XsSize numValues)
{
	XsSize i;
	for (i=0; i<numValues; i++)
	{
		switch (dataIdentifier & XDI_SubFormatMask)
		{
		case XDI_SubFormatFloat:
#ifdef XSENS_SINGLE_PRECISION
			*dest++ = XsMessage_getDataFloat(thisPtr, offset);
#else
			*dest++ = convertFromFloat(XsMessage_getDataFloat(thisPtr, offset));
#endif
			offset += 4;
			break;

		case XDI_SubFormatDouble:
#ifdef XSENS_SINGLE_PRECISION
			*dest++ = convertToFloat(XsMessage_getDataDouble(thisPtr, offset));
#else
			*dest++ = XsMessage_getDataDouble(thisPtr, offset);
#endif
			offset += 8;
			break;

		case XDI_SubFormatFp1632:
#ifdef XSENS_SINGLE_PRECISION
			*dest++ = convertToFloat(XsMessage_getDataFP1632(thisPtr, offset));
#else
			*dest++ = XsMessage_getDataFP1632(thisPtr, offset);
#endif
			offset += 6;
			break;

		case XDI_SubFormatFp1220:
#ifdef XSENS_SINGLE_PRECISION
			*dest++ = convertToFloat(XsMessage_getDataF1220(thisPtr, offset));
#else
			*dest++ = XsMessage_getDataF1220(thisPtr, offset);
#endif
			offset += 4;
			break;

		default:
			*dest++ = 0;
			break;
		}
	}
}

/*! \brief Write a number of floating/fixed point values into to the data buffer, conversion depends on data identifier

	\param dataIdentifier Data Identifier
	\param data		The data array to be written to the buffer.
	\param offset Offset in the data buffer from where to start writing.
	\param numValues number of values to be written
*/
void XsMessage_setDataRealValuesById(XsMessage* thisPtr, XsDataIdentifier dataIdentifier, XsReal const*data, XsSize offset, XsSize numValues)
{
	XsSize i;
	for (i=0; i<numValues; i++)
	{
		switch (dataIdentifier & XDI_SubFormatMask)
		{
		case XDI_SubFormatFloat:
			XsMessage_setDataFloat(thisPtr, convertToFloat(data[i]), offset);
			offset += 4;
			break;

		case XDI_SubFormatDouble:
			XsMessage_setDataDouble(thisPtr, data[i], offset);
			offset += 8;
			break;

		case XDI_SubFormatFp1632:
			XsMessage_setDataFP1632(thisPtr, data[i], offset);
			offset += 6;
			break;

		case XDI_SubFormatFp1220:
			XsMessage_setDataF1220(thisPtr, data[i], offset);
			offset += 4;
			break;

		default:
			break;
		}
	}
}

/*! \brief Computes the checksum for the message */
uint8_t XsMessage_computeChecksum(XsMessage const* thisPtr)
{
	XsSize i, msgSize;
	uint8_t cs = 0;

	msgSize = XsMessage_getTotalMessageSize(thisPtr)-1;
	for (i = 1; i < msgSize; ++i)
		cs -= thisPtr->m_message.m_data[i];

	return cs;
}

/*! \brief Update the checksum for the message
*/
void XsMessage_recomputeChecksum(XsMessage* thisPtr)
{
	assert(thisPtr->m_checksum);
	thisPtr->m_checksum[0] = XsMessage_computeChecksum(thisPtr);
}

/*! \brief Returns non-zero if the checksum inside the message is correct for the message, zero otherwise
	\returns true (non-zero) if the checksum inside the message is correct, false (zero) otherwise
*/
int XsMessage_isChecksumOk(XsMessage const* thisPtr)
{
	assert(thisPtr->m_checksum);
	return thisPtr->m_checksum[0] == XsMessage_computeChecksum(thisPtr);
}

/*! \brief Returns a pointer to the message header for this message */
XsMessageHeader* XsMessage_getHeader(XsMessage* thisPtr)
{
	return (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
}

/*! \brief \returns a const pointer to the message header */
const XsMessageHeader* XsMessage_getConstHeader(XsMessage const* thisPtr)
{
	return (const XsMessageHeader*) (void*) thisPtr->m_message.m_data;
}

/*! \brief Test if this message is empty
	\returns non-zero if this message is empty, zero otherwise
*/
int XsMessage_empty(XsMessage const* thisPtr)
{
	XsMessageHeader* hdr = (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
	if (!hdr)
		return 1;
	return (hdr->m_messageId == 0 && hdr->m_busId == XS_BID_MASTER);
}

/*! \brief Resize the buffer of message to \a newSize bytes

	\param newSize the new size of the message payload buffer
*/
void XsMessage_resizeData(XsMessage* thisPtr, XsSize newSize)
{
	XsSize index, oldSize;
	XsByteArray old = XSBYTEARRAY_INITIALIZER;
	XsMessageHeader* oldHdr, *newHdr;
	uint8_t* oldData, *newData;

	oldSize = XsMessage_dataSize(thisPtr);
	if (oldSize == newSize)
		return;

	XsArray_swap(&thisPtr->m_message, &old);
	XsMessage_constructSized(thisPtr, newSize);

	newHdr = (XsMessageHeader*) thisPtr->m_message.m_data;
	oldHdr = (XsMessageHeader*) old.m_data;
	if (!oldHdr)		// our original message may have been empty / uninitialized
		return;

	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] += newHdr->m_busId;

	newHdr->m_busId = oldHdr->m_busId;
	newHdr->m_preamble = oldHdr->m_preamble;
	newHdr->m_messageId = oldHdr->m_messageId;

	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] -= newHdr->m_busId + newHdr->m_messageId;

	if (oldSize >= XS_EXTLENCODE)
		oldData = oldHdr->m_datlen.m_extended.m_data;
	else
		oldData = oldHdr->m_datlen.m_data;

	if (newSize >= XS_EXTLENCODE)
		newData = newHdr->m_datlen.m_extended.m_data;
	else
		newData = newHdr->m_datlen.m_data;

	if (oldSize > newSize)
		oldSize = newSize;	// speed up the loop
	for (index = 0; index < oldSize; ++index)
		newData[index] = oldData[index];

	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] -= byteSum(oldData, oldSize);

	XsArray_destruct(&old);
}

/*! \brief Set the bus id for this message to \a busId

	\param busId the bus identifier

	\sa XS_BID_BROADCAST XS_BID_MASTER XS_BID_MT
*/
void XsMessage_setBusId(XsMessage* thisPtr, uint8_t busId)
{
	XsMessageHeader* hdr;
	if (!thisPtr->m_message.m_data)
		XsMessage_construct(thisPtr);

	hdr = (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] += hdr->m_busId;
	hdr->m_busId = busId;
	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] -= hdr->m_busId;
}

/*! \brief Set the message id for this message to \a msgId

	\param msgId the message identifier
*/
void XsMessage_setMessageId(XsMessage* thisPtr, enum XsXbusMessageId msgId)
{
	XsMessageHeader* hdr;
	if (!thisPtr->m_message.m_data)
		XsMessage_construct(thisPtr);

	hdr = (XsMessageHeader*) (void*) thisPtr->m_message.m_data;
	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] += hdr->m_messageId;
	hdr->m_messageId = (uint8_t) msgId;
	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] -= hdr->m_messageId;
}

/*! \brief Create \a count bytes of empty space at \a offset in this message

	\param count the number of bytes to reserve
	\param offset the offset at which to allocate the space
*/
void XsMessage_insertData(XsMessage* thisPtr, XsSize count, XsSize offset)
{
	XsSize index, oldSize, newSize;
	XsByteArray old = XSBYTEARRAY_INITIALIZER;
	XsMessageHeader* oldHdr, *newHdr;
	uint8_t* oldData, *newData;

	if (!count)
		return;

	oldSize = XsMessage_dataSize(thisPtr);
	newSize = oldSize+count;

	if (newSize < offset+count)
		newSize = offset+count;

	XsArray_swap(&thisPtr->m_message, &old);
	XsMessage_constructSized(thisPtr, newSize);

	newHdr = (XsMessageHeader*) thisPtr->m_message.m_data;
	oldHdr = (XsMessageHeader*) old.m_data;
	if (!oldHdr)		// our original message may have been empty / uninitialized
		return;

	newHdr->m_busId = oldHdr->m_busId;
	newHdr->m_preamble = oldHdr->m_preamble;
	newHdr->m_messageId = oldHdr->m_messageId;

	if (oldSize >= XS_EXTLENCODE)
		oldData = &oldHdr->m_datlen.m_extended.m_data[0];
	else
		oldData = &oldHdr->m_datlen.m_data[0];

	if (newSize >= XS_EXTLENCODE)
		newData = &newHdr->m_datlen.m_extended.m_data[0];
	else
		newData = &newHdr->m_datlen.m_data[0];

	if (offset <= oldSize)
	{
		for (index = 0; index < offset; ++index)
			newData[index] = oldData[index];
	}
	else
	{
		for (index = 0; index < oldSize; ++index)
			newData[index] = oldData[index];
		for (; index < offset; ++index)
			newData[index] = 0;
	}
	for (; index < oldSize; ++index)
		newData[index+count] = oldData[index];

	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] -= byteSum(oldData, oldSize);

	XsArray_destruct(&old);
}

/*! \brief Remove \a count bytes of data from the message at \a offset

	\param count the number of bytes to remove
	\param offset the offset at which to remove the bytes
*/
void XsMessage_deleteData(XsMessage* thisPtr, XsSize count, XsSize offset)
{
	XsSize index, oldSize, newSize;
	XsByteArray old = XSBYTEARRAY_INITIALIZER;
	XsMessageHeader* oldHdr, *newHdr;
	uint8_t* oldData, *newData;

	oldSize = XsMessage_dataSize(thisPtr);
	if (!count || offset >= oldSize)
		return;

	if (offset + count >= oldSize)
	{
		XsMessage_resizeData(thisPtr, offset);
		return;
	}
	newSize = oldSize-count;

	XsArray_swap(&thisPtr->m_message, &old);
	XsMessage_constructSized(thisPtr, newSize);

	newHdr = (XsMessageHeader*) thisPtr->m_message.m_data;
	oldHdr = (XsMessageHeader*) old.m_data;
	if (!oldHdr)		// our original message may have been empty / uninitialized
		return;

	newHdr->m_busId = oldHdr->m_busId;
	newHdr->m_preamble = oldHdr->m_preamble;
	newHdr->m_messageId = oldHdr->m_messageId;

	if (oldSize >= XS_EXTLENCODE)
		oldData = oldHdr->m_datlen.m_extended.m_data;
	else
		oldData = oldHdr->m_datlen.m_data;

	if (newSize >= XS_EXTLENCODE)
		newData = newHdr->m_datlen.m_extended.m_data;
	else
		newData = newHdr->m_datlen.m_data;

	for (index = 0; index < offset; ++index)
		newData[index] = oldData[index];
	for (; index < newSize; ++index)
		newData[index] = oldData[index+count];

	if (thisPtr->m_autoUpdateChecksum)
		thisPtr->m_checksum[0] -= byteSum(newData, newSize);

	XsArray_destruct(&old);
}

/*! \brief Swap the contents of \a a and \a b

	\details This function swaps the internal buffers so no actual data is moved around.
	A result is that it won't work for unmanaged data such as fixed size vectors

	\param a the object to receive \a b's contents
	\param b the object to receive \a a's contents
*/
void XsMessage_swap(XsMessage* a, XsMessage* b)
{
	XsMessage tmp;

	*((uint8_t**)&tmp.m_checksum) = a->m_checksum;
	tmp.m_autoUpdateChecksum = a->m_autoUpdateChecksum;

	*((uint8_t**)&a->m_checksum) = b->m_checksum;
	a->m_autoUpdateChecksum = b->m_autoUpdateChecksum;

	*((uint8_t**)&b->m_checksum) = tmp.m_checksum;
	b->m_autoUpdateChecksum = tmp.m_autoUpdateChecksum;

	XsArray_swap(&a->m_message, &b->m_message);
}

/*! \brief Compare the contents of the messages \a a and \a b, returning non-0 if they are different
*/
int XsMessage_compare(XsMessage const* a, XsMessage const* b)
{
	return XsArray_compare(&a->m_message, &b->m_message);
}

/*! \brief Return a string containing the first \a maxBytes bytes of the message in hex format
	\param maxBytes the maximum number of bytes to include in the string, when set to 0, the full message will be used
	\param resultValue the resulting string
*/
void XsMessage_toHexString(XsMessage const* thisPtr, XsSize maxBytes, XsString* resultValue)
{
	char* s;
	XsSize i = XsMessage_getTotalMessageSize(thisPtr);
	if (maxBytes == 0 || maxBytes > i)
		maxBytes = i;

	if (maxBytes)
	{
		XsString_resize(resultValue, maxBytes*3-1);
		s = (char*) resultValue->m_data;
		for (i = 0; i < maxBytes-1; ++i)
			sprintf(s+(i*3), "%02X ", (unsigned int) ((uint8_t const*) thisPtr->m_message.m_data)[i]);
		sprintf(s+((maxBytes-1)*3), "%02X", (unsigned int) ((uint8_t const*) thisPtr->m_message.m_data)[maxBytes-1]);
	}
	else
		XsString_resize(resultValue, 0);
}

/*! @} */
