/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>
#include <iosfwd>
#include <cstring>  // memset()
#include <mrpt/obs/gnss_messages_type_list.h>

namespace mrpt
{
namespace obs
{
/** GNSS (GPS) data structures, mainly for use within mrpt::obs::CObservationGPS
 */
namespace gnss
{
/** Pure virtual base for all message types. \sa mrpt::obs::CObservationGPS  */
struct gnss_message
{
	/** Type of GNSS message */
	gnss_message_type_t message_type;

	gnss_message(gnss_message_type_t msg_type_id) : message_type(msg_type_id) {}
	/** Save to binary stream. Launches an exception upon error */
	void writeToStream(mrpt::serialization::CArchive& out) const;
	/** Load from binary stream into this existing object. Launches an exception
	 * upon error. */
	void readFromStream(mrpt::serialization::CArchive& in);

	bool isOfType(const gnss_message_type_t type_id) const;
	template <class MSG_CLASS>
	bool isOfClass() const
	{
		return isOfType(MSG_CLASS::msg_type);
	}

	/** Load from binary stream and creates object detecting its type (class
	 * factory). Launches an exception upon error */
	static gnss_message* readAndBuildFromStream(
		mrpt::serialization::CArchive& in);
	/** Creates message \return nullptr on unknown msg type */
	static gnss_message* Factory(const gnss_message_type_t msg_id);
	/** Returns true if Factory() has a registered constructor for this msg type
	 */
	static bool FactoryKnowsMsgType(const gnss_message_type_t msg_id);

	/** Dumps the contents of the observation in a human-readable form to a
	 * given output stream \sa dumpToConsole() */
	virtual void dumpToStream(std::ostream& out) const = 0;
	/** Dumps the contents of the observation in a human-readable form to an
	 * std::ostream (set to std::cout to print to console) */
	void dumpToConsole(std::ostream& o) const;
	/** Dumps a header for getAllFieldValues() \return false if not implemented
	 * for this message type */
	virtual bool getAllFieldDescriptions(std::ostream& o) const
	{
		return false;
	}
	/** Dumps a line with the sequence of all field values (without a line feed
	 * at the end). \sa getAllFieldDescriptions() \return false if not
	 * implemented for this message type */
	virtual bool getAllFieldValues(std::ostream& o) const { return false; }
	/** Returns "NMEA_GGA", etc. */
	const std::string& getMessageTypeAsString() const;
	virtual ~gnss_message() = default;

   protected:
	/** Save to binary stream. Launches an exception upon error */
	virtual void internal_writeToStream(
		mrpt::serialization::CArchive& out) const = 0;
	/** Save to binary stream. Launches an exception upon error */
	virtual void internal_readFromStream(mrpt::serialization::CArchive& in) = 0;
};

/** A smart pointer to a GNSS message. \sa gnss_message,
 * mrpt::obs::CObservationGPS  */
struct gnss_message_ptr
{
   protected:
	gnss_message* ptr{nullptr};

   public:
	/** Ctor (default: nullptr pointer) */
	gnss_message_ptr();
	/** Makes a copy of the pointee */
	gnss_message_ptr(const gnss_message_ptr& o);
	/** Assigns a pointer. Memory now belongs to this class. */
	explicit gnss_message_ptr(const gnss_message* p);
	gnss_message_ptr& operator=(
		const gnss_message_ptr& o);  // Makes a copy of the pointee
	/** Dtor: it frees the pointee memory */
	virtual ~gnss_message_ptr();
	bool operator==(const gnss_message* o) const { return o == ptr; }
	bool operator==(const gnss_message_ptr& o) const { return o.ptr == ptr; }
	bool operator!=(const gnss_message* o) const { return o != ptr; }
	bool operator!=(const gnss_message_ptr& o) const { return o.ptr != ptr; }
	gnss_message*& get() { return ptr; }
	const gnss_message* get() const { return ptr; }
	gnss_message*& operator->()
	{
		ASSERT_(ptr);
		return ptr;
	}
	const gnss_message* operator->() const
	{
		ASSERT_(ptr);
		return ptr;
	}
	/** Replaces the pointee with a new pointer. Its memory now belongs to this
	 * object, do not free manually. */
	void set(gnss_message* p);
};

#define GNSS_MESSAGE_BINARY_BLOCK(DATA_PTR, DATA_LEN)                        \
   protected:                                                                \
	void internal_writeToStream(mrpt::serialization::CArchive& out)          \
		const override                                                       \
	{                                                                        \
		out << static_cast<uint32_t>(DATA_LEN);                              \
		out.WriteBuffer(DATA_PTR, DATA_LEN);                                 \
	}                                                                        \
	void internal_readFromStream(mrpt::serialization::CArchive& in) override \
	{                                                                        \
		uint32_t nBytesInStream;                                             \
		in >> nBytesInStream;                                                \
		ASSERT_EQUAL_(nBytesInStream, DATA_LEN);                             \
		in.ReadBuffer(DATA_PTR, DATA_LEN);                                   \
	}                                                                        \
                                                                             \
   public:

#define GNSS_BINARY_MSG_DEFINITION_START(_MSG_ID)                            \
	struct Message_##_MSG_ID : public gnss_message                           \
	{                                                                        \
		GNSS_MESSAGE_BINARY_BLOCK(&fields, sizeof(fields))                   \
		enum : uint32_t                                                      \
		{                                                                    \
			msg_type = _MSG_ID                                               \
		}; /* Static msg type (member expected by templates)*/               \
		Message_##_MSG_ID() : gnss_message((gnss_message_type_t)msg_type) {} \
		struct content_t                                                     \
		{
#define GNSS_BINARY_MSG_DEFINITION_MID                                       \
	content_t() { ::memset(this, 0, sizeof(*this)); }                        \
	}                                                                        \
	;                                                                        \
	content_t fields; /** Message content, accesible by individual fields */ \
	void dumpToStream(std::ostream& out) const override;

#define GNSS_BINARY_MSG_DEFINITION_MID_END \
	}                                      \
	;

#define GNSS_BINARY_MSG_DEFINITION_END \
	GNSS_BINARY_MSG_DEFINITION_MID     \
	GNSS_BINARY_MSG_DEFINITION_MID_END

// Pragma to ensure we can safely serialize some of these structures
#pragma pack(push, 1)

/** UTC (Coordinated Universal Time) time-stamp structure for GPS messages. \sa
 * mrpt::obs::CObservationGPS */
struct UTC_time
{
	uint8_t hour{0};
	uint8_t minute{0};
	double sec{0};

	UTC_time();
	/** Build an MRPT timestamp with the hour/minute/sec of this structure and
	 * the date from the given timestamp. */
	mrpt::system::TTimeStamp getAsTimestamp(
		const mrpt::system::TTimeStamp& date) const;
	bool operator==(const UTC_time& o) const
	{
		return hour == o.hour && minute == o.minute && sec == o.sec;
	}
	bool operator!=(const UTC_time& o) const
	{
		return hour != o.hour || minute != o.minute || sec != o.sec;
	}
	/** Save to binary stream. Launches an exception upon error */
	void writeToStream(mrpt::serialization::CArchive& out) const;
	/** Save to binary stream. Launches an exception upon error */
	void readFromStream(mrpt::serialization::CArchive& in);
};

#pragma pack(pop)  // End of pack = 1
}  // namespace gnss
}  // namespace obs
}  // namespace mrpt
