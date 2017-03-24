/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef _CMTMESSAGE_H_2006_05_24
#define _CMTMESSAGE_H_2006_05_24

#ifndef _CMT_MONOLITHIC
#	include "cmtdef.h"
#	include "cmt1.h"
#endif

namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
/*! \brief Compute the checksum of the given byte string.
*/
uint8_t computeChecksum(const uint8_t* buffer, uint32_t length);

//////////////////////////////////////////////////////////////////////////////////////////
/* different alignment commands for gcc / MSVS, the structure needs to be 1-byte aligned.
*/
#ifdef _MSC_VER
	#pragma pack(push, 1)
#endif
/*! \brief A message header.

	DO NOT REORDER THE MEMBERS!
*/
struct MessageHeader {
	uint8_t m_preamble;
	uint8_t m_busId;
	uint8_t m_messageId;
	uint8_t m_length;
	union _mdl {
		struct _mextd {
			struct _mlen {
				uint8_t m_high;
				uint8_t m_low;
			} m_length;
			uint8_t m_data[CMT_MAXDATALEN];
		} m_extended;
		uint8_t m_data[254];
	} m_datlen;
}
// different alignment commands for gcc / MSVS
#ifdef _MSC_VER
	;
	#pragma pack(pop)
#else
	/*! \cond NODOXYGEN */	__attribute__((__packed__))	/*! \endcond */;
#endif

#if defined(_DEBUG) && defined(_CMT_CHECK_MSG_DATA_INTEGRITY)
	#define CMT_CHECKVAR	uint32_t m_checkvar;
	#define CMT_CHECKVAL	10101010
	#define CMT_CHECKASSIGN	m_checkvar = CMT_CHECKVAL;
	#define CMT_CHECKASSERT	if (m_checkvar != CMT_CHECKVAL) throw "Message assertion failed!";
#else
	#define CMT_CHECKVAR
	#define CMT_CHECKVAL
	#define CMT_CHECKASSIGN
	#define CMT_CHECKASSERT
#endif

#define swapEndian16(src) (((src) >> 8) | ((src) << 8))
#define swapEndian32(src) (((src) >> 24) | ((src) >> 8 & 0xFF00) | ((src) << 8 & 0xFF0000) | ((src) << 24))

//////////////////////////////////////////////////////////////////////////////////////////
//! \brief Class for storing a single message.
class Message {
protected:
	CMT_CHECKVAR

		//! The message header is the data buffer with interpretation
	MessageHeader* m_buffer;
		//! The checksum in the m_data or m_extendedData buffer
	uint8_t* m_checksum;
		//! The maximum size of the message, including header and footer
	uint32_t m_maxLength;

		//! Internal checksum computation
	uint8_t calcChecksum(void) const
	{ return computeChecksum(((uint8_t*)m_buffer) + 1, getTotalMessageSize()-2); }
		//! Internal function to get the start of the data buffer.
	uint8_t* getDataStart(void) const;

public:
	bool m_autoUpdateChecksum;

	/*! \brief Create a Message object with the given data length and message Id.

		The function allocates enough memory to hold an entire message with the given
		data length.
		\param msgId	The message Id that will be assigend to the m_messageId field.
		\param length	The length of the data in the message. This value is stored in
						\c m_createdLength as well as \c m_length or \c m_extendedLength.
		\param maxLength	The maximum data length that can be stored in the structure.
	*/
	Message(const uint8_t msgId = 0, const uint16_t length = 0,	const uint16_t maxLength = CMT_MAXMSGLEN);

	/*! \brief Create a message from the given source string

		This is done through a simple memory copy. The number of bytes copied is taken
		from the data in the message (so the message is interpreted first).
		Note that this does NOT recompute the checksum, nor is it checked.

		\param source		The source string containing message data
		\param size			The size of the source string
		\param maxLength	The maximum data length that can be stored in the structure.
	*/
	Message(const uint8_t* source, const uint16_t size, const uint16_t maxLength = CMT_MAXMSGLEN);

	Message(const Message& src);

		//! Destructor
	~Message();

		//! Clear all data in the message
	void clear(void);

		//! Return the current value of the m_busId field.
	uint8_t getBusId(void) const { return m_buffer->m_busId; }
	/*! \brief Return a pointer to the data buffer.

		\param offset An optional offset in the data buffer from where to start reading.
	*/
	uint8_t* getDataBuffer(const uint16_t offset = 0)
		{ return &((getDataStart())[offset]); }
	const uint8_t* getDataBuffer(const uint16_t offset = 0) const
		{ return &((getDataStart())[offset]); }
	/*! \brief Return the current value of the data as an unsigned byte (8 bits).

		\param offset An optional offset in the data buffer from where to start reading.
	*/
	uint8_t getDataByte(const uint16_t offset = 0) const
		{ return (getDataStart()[offset]); }
	/*! \brief Return the current value of the data as a double (64 bits).

		\param offset An optional offset in the data buffer from where to start reading.
	*/
	double getDataDouble(const uint16_t offset=0) const;
	/*! \brief Return the current value of the data as a float (32 bits).

		\param offset An optional offset in the data buffer from where to start reading.
	*/
	float getDataFloat(const uint16_t offset=0) const;
	/*! \brief Return the current value of the data as a double, converting it from FP 12.20

		\param offset An optional offset in the data buffer from where to start reading.
	*/
	double getDataF1220(const uint16_t offset=0) const;
	/*! \brief Return the current value of the data as a double, converting it from FP 16.32

		\param offset An optional offset in the data buffer from where to start reading.
	*/
	double getDataFP1632(const uint16_t offset=0) const;
	/*! \brief Return current data value as double, conversion depends on outputSettings.

		\param outputSettings MT output settings
		\param offset An optional offset in the data buffer from where to start reading.
	*/
	double getDataFPValue(const uint64_t outputSettings, const uint16_t offset = 0) const;
	/*! \brief Return current data values as double, conversion depends on outputSetting.

		\param dest destination array
		\param outputSettings MT output settings
		\param offset offset in the data buffer from where to start reading.
		\param numValues number of values to be read
	*/
	void getDataFPValue(double *dest, const uint64_t outputSettings, uint16_t offset, const int16_t numValues) const;
	/*! \brief Return the current value of the data as an uint32_t (32 bits).

		\param offset An optional offset in the data buffer from where to start reading.
	*/
	uint32_t getDataLong(const uint16_t offset=0) const;
	/*! \brief Return the current value of the data as an uint16_t (16 bits).

		\param offset An optional offset in the data buffer from where to start reading.
	*/
	uint16_t getDataShort(const uint16_t offset=0) const;
		//! Return the length of the data part of the message.
	uint16_t getDataSize(void) const;
		//! Return the current value of the m_messageId field.
	uint8_t getMessageId(void) const { return m_buffer->m_messageId; }
	/*! \brief Return the start of the message buffer.

		The function returns the address of the \c m_preamble member.
	*/
	const uint8_t* getMessageStart(void) const
		{ return (uint8_t*) m_buffer; }
	/*! \brief Return the length of the message buffer.

		The function returns the total size of the message, including the checksum. This
		is in effect the number of bytes that would be transferred if the message were to
		be sent over a communications channel.
	*/
	uint16_t getTotalMessageSize(void) const;
		//! Compute the checksum and compare it with the stored checksum. Equal is ok.
	bool isChecksumOk(void) const;
	/*! \brief Read the entire message from the given source string

		This is done through a simple memory copy. The number of bytes copied is \c
		m_createdLength.
		\param source		The source string containing message data
		\param size			The size of the source string
	*/
	XsensResultValue loadFromString(const uint8_t* source, const uint16_t size);
	/*! \brief Compute the checksum field and fill it.

		The checksum field should normally be correct at all times, but if you have
		somehow managed to mess it up, this function can be used to recompute it.
	*/
	void recomputeChecksum(void) { m_checksum[0] = calcChecksum(); }
		//! Resize the data area to the given size
	void resizeData(const uint16_t newSize);
		//! Set the new value of the m_busId field and update the checksum.
	void setBusId(const uint8_t busId);
	/*! \brief Write a string of bytes into the data buffer.

		\param data		The data to write to the buffer.
		\param offset	An optional offset in the data buffer from where to start writing.
		\param count	An optional number of bytes to write, if set to 0 (not set), as
						many bytes as will fit into the buffer from the given point will
						be written.
	*/
	void setDataBuffer(const uint8_t* data, const uint16_t offset = 0,
								const uint16_t count = 0);
	/*! \brief Write an unsigned byte (8 bits) into the data buffer.

		\param data		The data to write to the buffer.
		\param offset	An optional offset in the data buffer from where to start writing.
	*/
	void setDataByte(const uint8_t data, const uint16_t offset = 0);
	/*! \brief Write a double (64 bits) into the data buffer.

		\param data		The data to write to the buffer.
		\param offset An optional offset in the data buffer from where to start writing.
	*/
	void setDataDouble(const double data, const uint16_t offset=0);
	/*! \brief Write a float (32 bits) into the data buffer.

		\param data		The data to write to the buffer.
		\param offset An optional offset in the data buffer from where to start writing.
	*/
	void setDataFloat(const float data, const uint16_t offset = 0);
	/*! \brief Write a double (64 bits) into the data buffer, after converting it to F1220.

		\param data		The data to write to the buffer.
		\param offset	An optional offset in the data buffer from where to start writing.
	*/
	void setDataF1220(const double data, const uint16_t offset = 0);
	/*! \brief Write a double (64 bits) into the data buffer, after converting it to FP1632.

		\param data		The data to write to the buffer.
		\param offset	An optional offset in the data buffer from where to start writing.
	*/
	void setDataFP1632(const double data, const uint16_t offset = 0);
	/*! \brief Write a floating/fixed point value into to the data buffer, conversion depends on outputSettings

		\param outputSettings MT output settings
		\param data		The data to write to the buffer.
		\param offset An optional offset in the data buffer from where to start writing.
	*/
	void setDataFPValue(const uint64_t outputSettings, const double data, const uint16_t offset = 0);
	/*! \brief Write a floating/fixed point value into to the data buffer, conversion depends on outputSettings

		\param outputSettings MT output settings
		\param data		The data array to be written to the buffer.
		\param offset Offset in the data buffer from where to start writing.
		\param numValues number of values to be written
	*/
	void setDataFPValue(const uint64_t outputSettings, const double *data, uint16_t offset, const uint16_t numValues);
	/*! \brief Write an uint32_t (32 bits) into the data buffer.

		\param data		The data to write to the buffer.
		\param offset An optional offset in the data buffer from where to start writing.
	*/
	void setDataLong(const uint32_t data, const uint16_t offset = 0);
	/*! \brief Write an uint16_t (16 bits) into the data buffer.

		\param data		The data to write to the buffer.
		\param offset An optional offset in the data buffer from where to start writing.
	*/
	void setDataShort(const uint16_t data, const uint16_t offset = 0);
		//! Set the new value of the m_messageId field and update the checksum.
	void setMessageId(const uint8_t msgId);

		//! Copy message src into this
	void operator = (const Message& src);

		//! Remove a number of bytes from the message (this will reduce the message size)
	void deleteData(uint16_t size, uint16_t offset = 0);
		//! Insert a number of bytes into the message (this will increase the message size)
	void insertData(uint16_t size, uint16_t offset = 0);
};

} // end of xsens namespace

#endif	// _CMTMESSAGE_H_2006_05_24
