/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/utils/CStream.h>
#include <mrpt/system/datetime.h>
#include <iostream>
#include <mrpt/obs/link_pragmas.h>
#include <mrpt/obs/gnss_messages_type_list.h>

namespace mrpt {
namespace obs {
/** GNSS (GPS) data structures, mainly for use within mrpt::obs::CObservationGPS */
namespace gnss {

/** Pure virtual base for all message types. \sa mrpt::obs::CObservationGPS  */
struct OBS_IMPEXP gnss_message {
	gnss_message_type_t  message_type; //!< Type of GNSS message

	gnss_message(gnss_message_type_t msg_type_id) : message_type(msg_type_id) {}
	void writeToStream(mrpt::utils::CStream &out) const; //!< Save to binary stream. Launches an exception upon error
	void readFromStream(mrpt::utils::CStream &in); //!< Load from binary stream into this existing object. Launches an exception upon error.

	bool isOfType(const gnss_message_type_t type_id) const;
	template <class MSG_CLASS> 
	bool isOfClass() const { return isOfType(MSG_CLASS::msg_type); }

	static gnss_message* readAndBuildFromStream(mrpt::utils::CStream &in); //!< Load from binary stream and creates object detecting its type (class factory). Launches an exception upon error
	static gnss_message* Factory(const gnss_message_type_t msg_id); //!< Creates message

	virtual void dumpToStream( mrpt::utils::CStream &out ) const = 0; //!< Dumps the contents of the observation in a human-readable form to a given output stream \sa dumpToConsole()
	void dumpToConsole(std::ostream &o = std::cout) const; //!< Dumps the contents of the observation in a human-readable form to an std::ostream (default=console)
    virtual ~gnss_message() {}
protected:
	virtual void internal_writeToStream(mrpt::utils::CStream &out) const = 0; //!< Save to binary stream. Launches an exception upon error
	virtual void internal_readFromStream(mrpt::utils::CStream &in) = 0; //!< Save to binary stream. Launches an exception upon error
};

/** A smart pointer to a GNSS message. \sa gnss_message, mrpt::obs::CObservationGPS  */
struct OBS_IMPEXP gnss_message_ptr 
{
protected:
	gnss_message *ptr;
public:
	gnss_message_ptr(); //!< Ctor (default: NULL pointer)
	gnss_message_ptr(const gnss_message_ptr &o); //!< Makes a copy of the pointee
	/** Assigns a pointer. Memory now belongs to this class. */
	explicit gnss_message_ptr(const gnss_message* p);
	gnss_message_ptr &operator =(const gnss_message_ptr&o); // Makes a copy of the pointee
	virtual ~gnss_message_ptr(); //!< Dtor: it frees the pointee memory
	bool operator == ( const gnss_message *o ) const { return o==ptr; }
	bool operator == ( const gnss_message_ptr &o )const { return o.ptr==ptr; }
	bool operator != ( const gnss_message *o )const { return o!=ptr; }
	bool operator != ( const gnss_message_ptr &o )const { return o.ptr!=ptr; }
	gnss_message*& get() { return ptr; }
	const gnss_message* get()const { return ptr; }
	gnss_message *& operator ->() { ASSERT_(ptr); return ptr; }
	const gnss_message * operator ->() const  { ASSERT_(ptr); return ptr; }
	void set(gnss_message* p); //!< Replaces the pointee with a new pointer. Its memory now belongs to this object, do not free manually.
};


struct OBS_IMPEXP gnss_message_binary_block : public gnss_message {
	gnss_message_binary_block(gnss_message_type_t msg_type_id,uint32_t data_len, void* data_ptr) : gnss_message(msg_type_id),m_content_len(data_len),m_content_ptr(data_ptr) {}
protected:
	void internal_writeToStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE;
	void internal_readFromStream(mrpt::utils::CStream &in) MRPT_OVERRIDE;
private:
	const uint32_t m_content_len;
	void *         m_content_ptr;
};

// Pragma to ensure we can safely serialize some of these structures
#pragma pack(push,1)

/** UTC (Coordinated Universal Time) time-stamp structure for GPS messages. \sa mrpt::obs::CObservationGPS */
struct OBS_IMPEXP UTC_time
{
	uint8_t hour;
	uint8_t minute;
	double  sec;

	UTC_time();
	mrpt::system::TTimeStamp getAsTimestamp(const mrpt::system::TTimeStamp &date) const; //!< Build an MRPT timestamp with the hour/minute/sec of this structure and the date from the given timestamp.
	bool operator == (const UTC_time& o) const { return hour==o.hour && minute==o.minute && sec==o.sec; }
	bool operator != (const UTC_time& o) const { return hour!=o.hour || minute!=o.minute || sec!=o.sec; }
	void writeToStream(mrpt::utils::CStream &out) const; //!< Save to binary stream. Launches an exception upon error
	void readFromStream(mrpt::utils::CStream &in); //!< Save to binary stream. Launches an exception upon error
};

#pragma pack(pop) // End of pack = 1
} } } // End of namespaces

