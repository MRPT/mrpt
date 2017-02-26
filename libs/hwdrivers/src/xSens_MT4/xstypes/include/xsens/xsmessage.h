/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSMESSAGE_H
#define XSMESSAGE_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xsbytearray.h"
#include "xsdataidentifier.h"
#include "xsxbusmessageid.h"

struct XsMessage;
struct XsMessageHeader;

#ifdef __cplusplus
extern "C" {
#else
#define XSMESSAGE_INITIALIZER	{ XSBYTEARRAY_INITIALIZER, 1, 0 }
typedef struct XsMessage XsMessage;
typedef struct XsMessageHeader XsMessageHeader;
#endif

XSTYPES_DLL_API void XsMessage_construct(XsMessage* thisPtr);
XSTYPES_DLL_API void XsMessage_constructSized(XsMessage* thisPtr, XsSize dataSize);
XSTYPES_DLL_API void XsMessage_copyConstruct(XsMessage* thisPtr, XsMessage const* src);
XSTYPES_DLL_API void XsMessage_assign(XsMessage* thisPtr, XsSize dataSize);
XSTYPES_DLL_API void XsMessage_load(XsMessage* thisPtr, XsSize msgSize, unsigned char const* src);
XSTYPES_DLL_API void XsMessage_destruct(XsMessage* thisPtr);
XSTYPES_DLL_API void XsMessage_copy(XsMessage* copy, XsMessage const* src);
XSTYPES_DLL_API void XsMessage_swap(XsMessage* a, XsMessage* b);
XSTYPES_DLL_API XsSize XsMessage_dataSize(XsMessage const* thisPtr);
XSTYPES_DLL_API const uint8_t* XsMessage_constData(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API const uint8_t* XsMessage_getMessageStart(XsMessage const* thisPtr);
XSTYPES_DLL_API XsSize XsMessage_getTotalMessageSize(XsMessage const* thisPtr);
XSTYPES_DLL_API uint8_t XsMessage_getDataByte(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API uint16_t XsMessage_getDataShort(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API uint32_t XsMessage_getDataLong(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API float XsMessage_getDataFloat(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API double XsMessage_getDataDouble(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API double XsMessage_getDataF1220(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API double XsMessage_getDataFP1632(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API const uint8_t* XsMessage_getDataBuffer(XsMessage const* thisPtr, XsSize offset);
XSTYPES_DLL_API void XsMessage_setDataByte(XsMessage* thisPtr, uint8_t value, XsSize offset);
XSTYPES_DLL_API void XsMessage_setDataShort(XsMessage* thisPtr, uint16_t value, XsSize offset);
XSTYPES_DLL_API void XsMessage_setDataLong(XsMessage* thisPtr, uint32_t value, XsSize offset);
XSTYPES_DLL_API void XsMessage_setDataFloat(XsMessage* thisPtr, float value, XsSize offset);
XSTYPES_DLL_API void XsMessage_setDataDouble(XsMessage* thisPtr, double value, XsSize offset);
XSTYPES_DLL_API void XsMessage_setDataF1220(XsMessage* thisPtr, double value, XsSize offset);
XSTYPES_DLL_API void XsMessage_setDataFP1632(XsMessage* thisPtr, double value, XsSize offset);
XSTYPES_DLL_API void XsMessage_setDataBuffer(XsMessage* thisPtr, const uint8_t* buffer, XsSize size, XsSize offset);
XSTYPES_DLL_API uint8_t XsMessage_computeChecksum(XsMessage const* thisPtr);
XSTYPES_DLL_API void XsMessage_recomputeChecksum(XsMessage* thisPtr);
XSTYPES_DLL_API int XsMessage_isChecksumOk(XsMessage const* thisPtr);
XSTYPES_DLL_API XsMessageHeader* XsMessage_getHeader(XsMessage* );
XSTYPES_DLL_API const XsMessageHeader* XsMessage_getConstHeader(XsMessage const* thisPtr);
XSTYPES_DLL_API int XsMessage_empty(XsMessage const* thisPtr);
XSTYPES_DLL_API void XsMessage_resizeData(XsMessage* thisPtr, XsSize newSize);
XSTYPES_DLL_API void XsMessage_setBusId(XsMessage* thisPtr, uint8_t busId);
XSTYPES_DLL_API void XsMessage_setMessageId(XsMessage* thisPtr, XsXbusMessageId msgId);
XSTYPES_DLL_API void XsMessage_insertData(XsMessage* thisPtr, XsSize count, XsSize offset);
XSTYPES_DLL_API void XsMessage_deleteData(XsMessage* thisPtr, XsSize count, XsSize offset);
XSTYPES_DLL_API void XsMessage_getDataFPValues(XsMessage const* thisPtr, uint64_t outputSettings, double *dest, XsSize offset, XsSize numValues);
XSTYPES_DLL_API void XsMessage_setDataFPValues(XsMessage* thisPtr, uint64_t outputSettings, double const *data, XsSize offset, XsSize numValues);
XSTYPES_DLL_API void XsMessage_getDataFPValuesById(XsMessage const* thisPtr, XsDataIdentifier dataIdentifier, double *dest, XsSize offset, XsSize numValues);
XSTYPES_DLL_API void XsMessage_setDataFPValuesById(XsMessage* thisPtr, XsDataIdentifier dataIdentifier, double const *data, XsSize offset, XsSize numValues);
XSTYPES_DLL_API int XsMessage_compare(XsMessage const* a, XsMessage const* b);

#ifdef __cplusplus
} // extern "C"
#endif

#define XS_PREAMBLE           0xFA
#define XS_EXTLENCODE         0xFF

#define XS_LEN_MSGHEADER      4
#define XS_LEN_MSGEXTHEADER   6
#define XS_LEN_MSGHEADERCS    5
#define XS_LEN_MSGEXTHEADERCS 7
#define XS_LEN_CHECKSUM       1
#define XS_LEN_UNSIGSHORT     2
#define XS_LEN_UNSIGINT       4
#define XS_LEN_FLOAT          4

// Maximum message/data length
#define XS_MAXDATALEN         (8192-XS_LEN_MSGEXTHEADERCS)
#define XS_MAXSHORTDATALEN    254
#define XS_MAXMSGLEN          (XS_MAXDATALEN+XS_LEN_MSGEXTHEADERCS)
#define XS_MAXSHORTMSGLEN     (XS_MAXSHORTDATALEN+XS_LEN_MSGHEADERCS)
#define XS_MAXGARBAGE         (XS_MAXMSGLEN+1)

//////////////////////////////////////////////////////////////////////////////////////////
// different alignment commands for gcc / MSVS, the structure needs to be 1-byte aligned.
#ifdef _MSC_VER
	#pragma pack(push, 1)
	#ifndef	PACK_POST
	#define PACK_POST
	#endif
#else
	#ifndef	PACK_POST
	#define PACK_POST __attribute__((__packed__))
	#endif
#endif
/*! \brief A message header
	\details This structure is used to interpret the header of a message.
*/
struct XsMessageHeader {
	uint8_t m_preamble;  //!< \brief The message preamble (always 0xFA)
	uint8_t m_busId;     //!< \brief The bus ID \sa XS_BID_MASTER XS_BID_BROADCAST XS_BID_MT
	uint8_t m_messageId; //!< \brief The message ID \sa XsXbusMessageId
	uint8_t m_length;    //!< \brief The length of the message \details A length of 255 means extended length is used
	//lint --e{18}
	//! \brief Contains optional extended length of message and first byte of data buffer
	union LengthData {
		//! \brief Contains extended length information and first byte of data buffer if normal length is 255
		struct ExtendedLength {
			//! \brief The high and low byte of the extended length
			struct ExtendedParts {
				uint8_t m_high;	//!< \brief High byte of extended length
				uint8_t m_low;	//!< \brief Low byte of extended length
			} m_length;			//!< \brief Extended length, only valid if normal length is 255
			uint8_t m_data[1];	//!< \brief The first byte of the data buffer, the data buffer is always at least 1 byte since it has to contain the checksum, but it can be bigger.
		} m_extended;			//!< \brief The extended length, only valid if normal length is 255
		uint8_t m_data[1];		//!< \brief The first byte of the data buffer if length < 255, the data buffer is always at least 1 byte since it has to contain the checksum, but it can be bigger.
	} m_datlen;	//!< \brief Data or length and data
} PACK_POST;	//lint !e659
#ifdef _MSC_VER
	#pragma pack(pop)
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//! \brief Structure for storing a single message.
struct XsMessage {
#ifdef __cplusplus
	/*! \brief Create a XsMessage object with the given data length and message Id.

		The function allocates enough memory to hold an entire message with the given
		data length.
		\param msgId		The message Id that will be assigned to the m_messageId field.
		\param dataLength	The length of the data in the message.
	*/
	explicit XsMessage(XsXbusMessageId msgId = XMID_InvalidMessage, XsSize dataLength = 0)
		: m_autoUpdateChecksum(1)
		, m_checksum(0)
	{
		XsMessage_constructSized(this, dataLength);
		XsMessage_setMessageId(this, msgId);
	}

	/*! \brief Create a message from the given source string

		This is done through a simple memory copy. The number of bytes copied is taken
		from the data in the message (so the message is interpreted first).
		Note that this does NOT recompute the checksum, nor is it checked.

		\param source		The source string containing message data
		\param size			The size of the source string
	*/
	XsMessage(const uint8_t* source, XsSize size)
		: m_autoUpdateChecksum(1)
		, m_checksum(0)
	{
		XsMessage_load(this, size, source);
	}

	//! \brief Copy constructor
	XsMessage(const XsMessage& src)
		: m_message(src.m_message)
		, m_autoUpdateChecksum(src.m_autoUpdateChecksum)
		, m_checksum(0)
	{
		updateChecksumPtr();
	}

	//! Destroy the message
	~XsMessage()
	{
		XsMessage_destruct(this);
	}

	//! \brief Clear all data in the message
	void clear(void)
	{
		XsMessage_destruct(this);
	}

	/*! \brief Test if this message is empty 
 
	  \returns true if this message is empty, false otherwise
	*/
	bool empty(void) const
	{
		return 0 != XsMessage_empty(this);
	}

	//! Return the busId header field.
	uint8_t getBusId(void) const
	{
		const XsMessageHeader* hdr = XsMessage_getConstHeader(this);
		if (!hdr)
			return 0;
		return hdr->m_busId;
	}

	/*! \copydoc XsMessage_constData
	*/
	const uint8_t* getDataBuffer(XsSize offset = 0) const
	{
		return XsMessage_constData(this, offset);
	}

	/*! \copydoc XsMessage_getDataByte
	*/
	uint8_t getDataByte(XsSize offset = 0) const
	{
		return XsMessage_getDataByte(this, offset);
	}

	/*! \copydoc XsMessage_getDataDouble
	*/
	double getDataDouble(XsSize offset = 0) const
	{
		return XsMessage_getDataDouble(this, offset);
	}

	/*! \copydoc XsMessage_getDataFloat
	*/
	float getDataFloat(XsSize offset = 0) const
	{
		return XsMessage_getDataFloat(this, offset);
	}

	/*! \copydoc XsMessage_getDataF1220
	*/
	double getDataF1220(XsSize offset = 0) const
	{
		return XsMessage_getDataF1220(this, offset);
	}

	/*! \copydoc XsMessage_getDataFP1632
	*/
	double getDataFP1632(XsSize offset = 0) const
	{
		return XsMessage_getDataFP1632(this, offset);
	}

	/*! \copydoc XsMessage_getDataLong
	*/
	uint32_t getDataLong(XsSize offset = 0) const
	{
		return XsMessage_getDataLong(this, offset);
	}

	/*! \copydoc XsMessage_getDataShort
	*/
	uint16_t getDataShort(XsSize offset = 0) const
	{
		return XsMessage_getDataShort(this, offset);
	}

	/*! \copydoc XsMessage_dataSize
	*/
	XsSize getDataSize(void) const
	{
		return XsMessage_dataSize(this);
	}

	//! Return the current value of the m_messageId field.
	XsXbusMessageId getMessageId(void) const
	{
		const XsMessageHeader* hdr = XsMessage_getConstHeader(this);
		if (!hdr)
			return XMID_InvalidMessage;
		return (XsXbusMessageId) hdr->m_messageId;
	}

	/*! \copydoc XsMessage_getMessageStart
	*/
	const uint8_t* getMessageStart(void) const
	{
		return XsMessage_getMessageStart(this);
	}

	/*!	\copydoc XsMessage_getTotalMessageSize
	*/
	XsSize getTotalMessageSize(void) const
	{
		return XsMessage_getTotalMessageSize(this);
	}

	/*! \copydoc XsMessage_isChecksumOk
	*/
	bool isChecksumOk(void) const
	{
		return 0 != XsMessage_isChecksumOk(this);
	}

	/*! \brief Initialize the %XsMessage with the data from \a src

	  \param msgSize the size of the data pointed to by src
	  \param src the data to load the message from

	  \returns true if the checksum of the loaded message is OK.
	*/
	bool loadFromString(const uint8_t* src, XsSize msgSize)
	{
		XsMessage_load(this, msgSize, src);
		return isChecksumOk();
	}

	/*! \copydoc XsMessage_recomputeChecksum
	*/
	void recomputeChecksum(void)
	{
		XsMessage_recomputeChecksum(this);
	}

	/*! \copydoc XsMessage_resizeData
	*/	
	void resizeData(XsSize newSize)
	{
		XsMessage_resizeData(this, newSize);
	}

	/*! \copydoc XsMessage_setBusId
	*/
	void setBusId(uint8_t busId)
	{
		XsMessage_setBusId(this, busId);
	}

	/*! \copydoc XsMessage_setDataBuffer
	*/
	void setDataBuffer(const uint8_t* buffer, XsSize size, XsSize offset = 0)
	{
		XsMessage_setDataBuffer(this, buffer, size, offset);
	}

	/*! \copydoc XsMessage_setDataByte
	*/
	void setDataByte(const uint8_t value, XsSize offset = 0)
	{
		XsMessage_setDataByte(this, value, offset);
	}

	/*! \copydoc XsMessage_setDataDouble
	*/
	void setDataDouble(const double value, XsSize offset=0)
	{
		XsMessage_setDataDouble(this, value, offset);
	}

	/*! \copydoc XsMessage_setDataFloat
	*/
	void setDataFloat(const float value, XsSize offset = 0)
	{
		XsMessage_setDataFloat(this, value, offset);
	}

	/*! \copydoc XsMessage_setDataF1220
	*/
	void setDataF1220(const double value, XsSize offset = 0)
	{
		XsMessage_setDataF1220(this, value, offset);
	}

	/*! \copydoc XsMessage_setDataFP1632
	*/
	void setDataFP1632(const double value, XsSize offset = 0)
	{
		XsMessage_setDataFP1632(this, value, offset);
	}

	//void setDataFPValue(const uint64_t outputSettings, const double data, const XsSize offset = 0);
	//void setDataFPValue(const uint64_t outputSettings, const double *data, XsSize offset, const uint16_t numValues);
	//void setDataFPValue(const XsDataIdentifier dataIdentifier, const double data, const XsSize offset = 0);
	//void setDataFPValue(const XsDataIdentifier dataIdentifier, const double *data, XsSize offset, const uint16_t numValues);
	
	/*! \copydoc XsMessage_setDataLong
	*/
	void setDataLong(const uint32_t value, XsSize offset = 0)
	{
		XsMessage_setDataLong(this, value, offset);
	}

	/*!	\copydoc XsMessage_setDataShort
	*/
	void setDataShort(const uint16_t value, XsSize offset = 0)
	{
		XsMessage_setDataShort(this, value, offset);
	}

	/*! \copydoc XsMessage_setMessageId
	*/
	void setMessageId(const XsXbusMessageId msgId)
	{
		XsMessage_setMessageId(this, msgId);
	}

	//! Copy message src into this
	XsMessage& operator = (const XsMessage& src)
	{
		if (this != &src)
			XsMessage_copy(this, &src);
		return *this;
	}

	/*! \copydoc XsMessage_deleteData */
	void deleteData(XsSize count, XsSize offset = 0)
	{
		XsMessage_deleteData(this, count, offset);
	}

	/*! \copydoc XsMessage_insertData */
	void insertData(XsSize count, XsSize offset = 0)
	{
		XsMessage_insertData(this, count, offset);
	}

	/*! \copydoc XsMessage_getDataFPValues */
	void getDataFPValue(uint64_t outputSettings, double *dest, XsSize offset = 0, XsSize numValues = 1) const
	{
		XsMessage_getDataFPValues(this, outputSettings, dest, offset, numValues);
	}

	/*! \brief Return current data value as a double(64 bits), after converting it from float, double, FP1632 or FP1220 depending on outputSettings

	  \param outputSettings the output settings containing the current floating/fixed point settings
	  \param offset the offset in the message payload at which to start reading

	  \returns the double precision representation of the value at \a offset
	*/
	double getDataFPValue(uint64_t outputSettings, XsSize offset = 0) const
	{
		double tmp;
		XsMessage_getDataFPValues(this, outputSettings, &tmp, offset, 1);
		return tmp;
	}

	/*! \copydoc XsMessage_setDataFPValues */
	void setDataFPValue(uint64_t outputSettings, const double *data, XsSize offset = 0, XsSize numValues = 1)
	{
		XsMessage_setDataFPValues(this, outputSettings, data, offset, numValues);
	}

	/*! \brief Write a floating/fixed point value into to the data buffer, conversion depends on outputSettings

		\param outputSettings MT output settings
		\param data	The data array to be written to the buffer.
		\param offset Offset in the data buffer from where to start writing.

		\sa setDataFPValue(uint64_t, const double *, XsSize, XsSize)
	*/
	void setDataFPValue(uint64_t outputSettings, double data, XsSize offset = 0)
	{
		XsMessage_setDataFPValues(this, outputSettings, &data, offset, 1);
	}

	/*! \copydoc XsMessage_getDataFPValuesById */
	void getDataFPValue(XsDataIdentifier dataIdentifier, double *dest, XsSize offset = 0, XsSize numValues = 1) const
	{
		XsMessage_getDataFPValuesById(this, dataIdentifier, dest, offset, numValues);
	}

	/*! \brief Return current data values as double, conversion depends on outputSetting

		\param dataIdentifier Data identifier containing data precision
		\param offset offset in the data buffer from where to start reading.

		\returns the current data value as double
	*/
	double getDataFPValue(XsDataIdentifier dataIdentifier, XsSize offset = 0) const
	{
		double tmp;
		XsMessage_getDataFPValuesById(this, dataIdentifier, &tmp, offset, 1);
		return tmp;
	}

	/*! \copydoc XsMessage_setDataFPValuesById */
	void setDataFPValue(XsDataIdentifier dataIdentifier, const double *data, XsSize offset = 0, XsSize numValues = 1)
	{
		XsMessage_setDataFPValuesById(this, dataIdentifier, data, offset, numValues);
	}

	/*! \brief Write a floating/fixed point value into to the data buffer, conversion depends on outputSettings

	  \param dataIdentifier Data Identifier
	  \param data		The data array to be written to the buffer.
	  \param offset Offset in the data buffer from where to start writing.
	*/
	void setDataFPValue(XsDataIdentifier dataIdentifier, double data, XsSize offset = 0)
	{
		XsMessage_setDataFPValuesById(this, dataIdentifier, &data, offset, 1);
	}

	/*! \brief Return true if \a other is identical to this */
	inline bool operator == (const XsMessage& other) const
	{
		if (this == &other)
			return true;
		return m_message == other.m_message;
	}

private:
	void updateChecksumPtr()
	{
		*((uint8_t**) &m_checksum) = &m_message[XsMessage_getTotalMessageSize(this)-1];
	}

#endif

	XsByteArray m_message;
	int m_autoUpdateChecksum;
	uint8_t* const m_checksum;	//! Points to the checksum to speed up automatic checksum updates
};

// some macros to help when constructing/parsing messages
#define swapEndian16(src) (((src) >> 8) | ((src) << 8))
#define swapEndian32(src) (((src) >> 24) | (((src) >> 8) & 0xFF00) | (((src) << 8) & 0xFF0000) | ((src) << 24))
#define swapEndian64(src) (((src >> 56) & 0xFFULL) | ((src >> 40) & 0xFF00ULL) | ((src >> 24) & 0xFF0000ULL) | ((src >> 8) & 0xFF000000ULL) | ((src << 8) & 0xFF00000000ULL) | ((src << 24) & 0xFF0000000000ULL) | ((src << 40) & 0xFF000000000000ULL) | ((src << 56)))

#endif	// file guard
