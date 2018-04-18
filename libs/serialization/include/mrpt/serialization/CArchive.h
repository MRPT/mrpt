/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>
#include <mrpt/config.h>  // MRPT_IS_BIG_ENDIAN
#include <mrpt/core/is_shared_ptr.h>
#include <mrpt/core/reverse_bytes.h>
#include <mrpt/serialization/CSerializable.h>
#include <vector>
#include <string>
#include <type_traits>  // remove_reference_t, is_polymorphic
#include <stdexcept>
#include <mrpt/rtti/variant.h>

namespace mrpt
{
namespace serialization
{
class CMessage;

/** Used in mrpt::serialization::CArchive */
class CExceptionEOF : public std::runtime_error
{
   public:
	CExceptionEOF(const std::string& s) : std::runtime_error(s) {}
};

/** Virtual base class for "archives": classes abstracting I/O streams.
 * This class separates the implementation details of serialization (in
 * CSerializable) and data storage (CArchive children: files, sockets,...).
 *
 * Two main sets of implementations are provided:
 * - archiveFrom: for MRPT mrpt::io::CArchive objects, and
 * - CArchiveStdIStream and CArchiveStdOStream: for std::istream and
 * std::ostream, respectively.
 *
 * \sa mrpt::io::CArchive, mrpt::serialization::CSerializable
 * \ingroup mrpt_serialization_grp
 */
class CArchive
{
   public:
	CArchive() {}
	virtual ~CArchive() {}
	/** @name Serialization API for generic "archives" I/O streams
	 * @{ */
	/** Reads a block of bytes from the stream into Buffer
	 *	\exception std::exception On any error, or if ZERO bytes are read.
	 *  \return The amound of bytes actually read.
	 * \note This method is endianness-dependent.
	 * \sa ReadBufferImmediate ; Important, see: ReadBufferFixEndianness,
	 */
	size_t ReadBuffer(void* Buffer, size_t Count);

	/** Reads a sequence of elemental datatypes, taking care of reordering their
	 *bytes from the MRPT stream standard (little endianness) to the format of
	 *the running architecture.
	 *  \param ElementCount The number of elements (not bytes) to read.
	 *  \param ptr A pointer to the first output element in an array (or
	 *std::vector<>, etc...).
	 *  \return The amound of *bytes* (not elements) actually read (under error
	 *situations, the last element may be invalid if the data stream abruptly
	 *ends).
	 *  Example of usage:
	 *  \code
	 *   uint32_t  N;
	 *   s >> N;
	 *   vector<float>  vec(N);
	 *   if (N)
	 *     s.ReadBufferFixEndianness<float>(&vec[0],N);
	 *  \endcode
	 *	\exception std::exception On any error, or if ZERO bytes are read.
	 * \sa ReadBufferFixEndianness, ReadBuffer
	 */
	template <typename T>
	size_t ReadBufferFixEndianness(T* ptr, size_t ElementCount)
	{
#if !MRPT_IS_BIG_ENDIAN
		// little endian: no conversion needed.
		return ReadBuffer(ptr, ElementCount * sizeof(T));
#else
		// big endian: convert.
		const size_t nread = ReadBuffer(ptr, ElementCount * sizeof(T));
		for (size_t i = 0; i < ElementCount; i++)
			mrpt::utils::reverseBytesInPlace(ptr[i]);
		return nread;
#endif
	}

	/** Writes a block of bytes to the stream from Buffer.
	 *	\exception std::exception On any error
	 *  \sa Important, see: WriteBufferFixEndianness
	 * \note This method is endianness-dependent.
	 */
	void WriteBuffer(const void* Buffer, size_t Count);

	/** Writes a sequence of elemental datatypes, taking care of reordering
	 * their bytes from the running architecture to MRPT stream standard (little
	 * endianness).
	 *  \param ElementCount The number of elements (not bytes) to write.
	 *  \param ptr A pointer to the first input element in an array (or
	 * std::vector<>, etc...).
	 *  Example of usage:
	 *  \code
	 *   vector<float>  vec = ...
	 *   uint32_t N = vec.size();
	 *   s << N
	 *   if (N)
	 *     s.WriteBufferFixEndianness<float>(&vec[0],N);
	 *  \endcode
	 *  \exception std::exception On any error
	 *  \sa WriteBuffer
	 */
	template <typename T>
	void WriteBufferFixEndianness(const T* ptr, size_t ElementCount)
	{
#if !MRPT_IS_BIG_ENDIAN
		// little endian: no conversion needed.
		return WriteBuffer(ptr, ElementCount * sizeof(T));
#else
		// big endian: the individual "<<" functions already convert endiannes
		for (size_t i = 0; i < ElementCount; i++) (*this) << ptr[i];
#endif
	}
	/** Read a value from a stream stored in a type different of the target
	 * variable, making the conversion via static_cast. Useful for coding
	 * backwards compatible de-serialization blocks */
	template <typename STORED_TYPE, typename CAST_TO_TYPE>
	void ReadAsAndCastTo(CAST_TO_TYPE& read_here)
	{
		STORED_TYPE var;
		(*this) >> var;
		read_here = static_cast<CAST_TO_TYPE>(var);
	}
	/** De-serialize a variable and returns it by value. */
	template <typename STORED_TYPE>
	STORED_TYPE ReadAs()
	{
		STORED_TYPE var;
		(*this) >> var;
		return var;
	}
	template <typename TYPE_TO_STORE, typename TYPE_FROM_ACTUAL>
	void WriteAs(const TYPE_FROM_ACTUAL& value)
	{
		(*this) << static_cast<TYPE_TO_STORE>(value);
	}
	/** Writes an object to the stream.
	 */
	void WriteObject(const CSerializable* o);
	void WriteObject(const CSerializable& o) { WriteObject(&o); }
	/** Reads an object from stream, its class determined at runtime, and
	 * returns a smart pointer to the object.
	 * \exception std::exception On I/O error or undefined class.
	 * \exception CExceptionEOF On an End-Of-File condition found
	 * at a correct place: an EOF that abruptly finishes in the middle of one
	 * object raises a plain std::exception instead.
	 */
	CSerializable::Ptr ReadObject() { return ReadObject<CSerializable>(); }
	/** Reads an object from stream, its class determined at runtime, and
	 * returns a smart pointer to the object. This version is similar to
	 * mrpt::make_aligned_shared<T>.
	 * \exception std::exception On I/O error or undefined class.
	 * \exception CExceptionEOF On an End-Of-File condition found
	 * at a correct place: an EOF that abruptly finishes in the middle of one
	 * object raises a plain std::exception instead.
	 */
	template <typename T>
	typename T::Ptr ReadObject()
	{
		CSerializable::Ptr obj;
		std::string strClassName;
		bool isOldFormat;
		int8_t version;
		internal_ReadObjectHeader(strClassName, isOldFormat, version);
		if (strClassName != "nullptr")
		{
			const mrpt::rtti::TRuntimeClassId* classId =
				mrpt::rtti::findRegisteredClass(strClassName);
			if (!classId)
				THROW_EXCEPTION_FMT(
					"Stored object has class '%s' which is not registered!",
					strClassName.c_str());
			obj.reset(dynamic_cast<CSerializable*>(classId->createObject()));
		}
		internal_ReadObject(
			obj.get() /* may be nullptr */, strClassName, isOldFormat,
			version);  // must be called to read the END FLAG byte
		if (!obj)
		{
			return typename T::Ptr();
		}
		else
		{
			return std::dynamic_pointer_cast<T>(obj);
		}
	}

   private:
	template <typename RET>
	RET ReadVariant_helper(CSerializable::Ptr& ptr)
	{
		throw std::runtime_error("Can't match variant type");
		return RET();
	}

	template <typename RET, typename T, typename... R>
	RET ReadVariant_helper(
		CSerializable::Ptr& ptr,
		std::enable_if_t<mrpt::is_shared_ptr<T>::value>* = nullptr)
	{
		if (IS_CLASS(ptr, typename T::element_type))
			return std::dynamic_pointer_cast<typename T::element_type>(ptr);
		return ReadVariant_helper<RET, R...>(ptr);
	}

	template <typename RET, typename T, typename... R>
	RET ReadVariant_helper(
		CSerializable::Ptr& ptr,
		std::enable_if_t<!mrpt::is_shared_ptr<T>::value>* = nullptr)
	{
		if (IS_CLASS(ptr, T)) return dynamic_cast<T&>(*ptr);
		return ReadVariant_helper<RET, R...>(ptr);
	}

   public:
	/** Reads a variant from stream, its class determined at runtime, and
	 * returns a variant to the object.
	 * To be compatible with the current polymorphic system this support smart
	 * pointer types.
	 * For pointer types, This will bind to the first possible pointer type.
	 * variant<CSerializable::Ptr, CRenderizable::Ptr>
	 * \exception std::exception On I/O error or undefined class.
	 * \exception CExceptionEOF On an End-Of-File condition found
	 * at a correct place: an EOF that abruptly finishes in the middle of one
	 * object raises a plain std::exception instead.
	 */
	template <typename... T>
	typename mrpt::rtti::variant<T...> ReadVariant()
	{
		CSerializable::Ptr obj;
		std::string strClassName;
		bool isOldFormat;
		int8_t version;
		internal_ReadObjectHeader(strClassName, isOldFormat, version);
		const mrpt::rtti::TRuntimeClassId* classId =
			mrpt::rtti::findRegisteredClass(strClassName);
		if (!classId)
			THROW_EXCEPTION_FMT(
				"Stored object has class '%s' which is not registered!",
				strClassName.c_str());
		if (strClassName != "nullptr")
		{
			obj.reset(dynamic_cast<CSerializable*>(classId->createObject()));
		}
		internal_ReadObject(obj.get(), strClassName, isOldFormat, version);
		if (!obj)
		{
			return mrpt::rtti::variant<T...>();
		}
		else
		{
			return ReadVariant_helper<mrpt::rtti::variant<T...>, T...>(obj);
		}
	}

	/** Writes a Variant to the stream.
	 */
	template <typename T>
	void WriteVariant(T t)
	{
		t.match([&](auto& o) { this->WriteObject(o); });
	}

	/** Reads a simple POD type and returns by value. Useful when `stream >>
	 * var;`
	 * cannot be used becuase of errors of misaligned reference binding.
	 * Use with macro `MRPT_READ_POD` to avoid typing the type T yourself.
	 * \note [New in MRPT 2.0.0]
	 * \note Write operator `s << var;` is safe for misaligned variables.
	 */
	template <typename T>
	T ReadPOD()
	{
		T ret;
		ReadBufferFixEndianness(&ret, 1);
		return ret;
	}

	/** Reads an object from stream, where its class must be the same
	 *    as the supplied object, where the loaded object will be stored in.
	 * \exception std::exception On I/O error or different class found.
	 * \exception CExceptionEOF On an End-Of-File condition found
	 * at a correct place: an EOF that abruptly finishes in the middle of one
	 * object raises a plain std::exception instead.
	 */
	void ReadObject(CSerializable* existingObj);

	/** Send a message to the device.
	 *  Note that only the low byte from the "type" field will be used.
	 *
	 *  For frames of size < 255 the frame format is an array of bytes in this
	 * order:
	 *  \code
	 *  <START_FLAG> <HEADER> <LENGTH> <BODY> <END_FLAG>
	 *  	<START_FLAG> 	= 0x69
	 *  	<HEADER> 		= A header byte
	 *  	<LENGHT>		= Number of bytes of BODY
	 *  	<BODY>			= N x bytes
	 *  	<END_FLAG>		= 0X96
	 *  Total length 	= 	<LENGTH> + 4
	 *  \endcode
	 *
	 *  For frames of size > 255 the frame format is an array of bytes in this
	 * order:
	 *  \code
	 *  <START_FLAG> <HEADER> <HIBYTE(LENGTH)> <LOBYTE(LENGTH)> <BODY>
	 * <END_FLAG>
	 *  	<START_FLAG> 	= 0x79
	 *  	<HEADER> 		= A header byte
	 *  	<LENGHT>		= Number of bytes of BODY
	 *  	<BODY>			= N x bytes
	 *  	<END_FLAG>		= 0X96
	 *  Total length 	= 	<LENGTH> + 5
	 *  \endcode
	 *
	 * \exception std::exception On communication errors
	 */
	void sendMessage(const CMessage& msg);

	/** Tries to receive a message from the device.
	 * \exception std::exception On communication errors
	 * \returns True if successful, false if there is no new data from the
	 * device (but communications seem to work fine)
	 * \sa The frame format is described in sendMessage()
	 */
	bool receiveMessage(CMessage& msg);

	/** Write a shared_ptr to a non-CSerializable object */
	template <class T, std::enable_if_t<!std::is_base_of<
		mrpt::serialization::CSerializable, T>::value>* = nullptr>
	CArchive& operator<<(const std::shared_ptr<T>& pObj)
	{
		if (pObj)
		{
			*this << mrpt::typemeta::TTypeName<T>::get();
			*this << *pObj;
		}
		else
		{
			*this << std::string("nullptr");
		}
		return *this;
	}

	/** Read a smart pointer to a non-CSerializable (POD,...) data type*/
	template <class T, std::enable_if_t<!std::is_base_of<
		mrpt::serialization::CSerializable, T>::value>* = nullptr>
	CArchive& operator>>(std::shared_ptr<T>& pObj)
	{
		std::string stored_name;
		*this >> stored_name;
		const std::string expected_name = mrpt::typemeta::TTypeName<T>::get();
		if (stored_name == std::string("nullptr"))
		{
			pObj.reset();
		}
		else
		{
			ASSERT_EQUAL_(expected_name, stored_name);
			pObj.reset(new T);
			*this >> *pObj;
		}
		return *this;
	}


	/** Write an object to a stream in the binary MRPT format. */
	CArchive& operator<<(const CSerializable& obj);
	CArchive& operator>>(CSerializable& obj);

	/** @} */

   protected:
	/** @name Virtual methods of the CArchive interface
	 * @{ */
	/** Writes a block of bytes.
	 * \exception std::exception On any error
	 * \return Number of bytes actually written.
	 */
	virtual size_t write(const void* buf, size_t len) = 0;
	/** Reads a block of bytes.
	 * \exception std::exception On any error, or if ZERO bytes are read.
	 * \return Number of bytes actually read if >0.
	 */
	virtual size_t read(void* buf, size_t len) = 0;
	/** @} */

	/** Read the object */
	void internal_ReadObject(
		CSerializable* newObj, const std::string& className, bool isOldFormat,
		int8_t version);

	/** Read the object Header*/
	void internal_ReadObjectHeader(
		std::string& className, bool& isOldFormat, int8_t& version);
};

// Note: write op accepts parameters by value on purpose, to avoid misaligned
// reference binding errors.
#define DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(T)  \
	CArchive& operator<<(CArchive& out, const T a); \
	CArchive& operator>>(CArchive& in, T& a)

// Definitions:
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(bool);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(uint8_t);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(int8_t);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(uint16_t);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(int16_t);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(uint32_t);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(int32_t);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(uint64_t);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(int64_t);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(float);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(double);
DECLARE_CArchive_READ_WRITE_SIMPLE_TYPE(long double);

#define MRPT_READ_POD(_STREAM, _VARIABLE)                                    \
	do                                                                       \
	{                                                                        \
		const std::remove_cv_t<std::remove_reference_t<decltype(_VARIABLE)>> \
			val = _STREAM.ReadPOD<std::remove_cv_t<                          \
				std::remove_reference_t<decltype(_VARIABLE)>>>();            \
		::memcpy(&_VARIABLE, &val, sizeof(val));                             \
	} while (0)

// Why this shouldn't be templatized?: There's a more modern system
// in MRPT that serializes any kind of vector<T>, deque<T>, etc... but
// to keep COMPATIBILITY with old serialized objects we must preserve
// the ones listed here:

// Write --------------------
CArchive& operator<<(CArchive& s, const char* a);
CArchive& operator<<(CArchive& s, const std::string& str);

CArchive& operator<<(CArchive&, const std::vector<int32_t>& a);
CArchive& operator<<(CArchive&, const std::vector<uint32_t>& a);
CArchive& operator<<(CArchive&, const std::vector<uint16_t>& a);
CArchive& operator<<(CArchive&, const std::vector<int16_t>& a);
CArchive& operator<<(CArchive&, const std::vector<uint32_t>& a);
CArchive& operator<<(CArchive&, const std::vector<int64_t>& a);
CArchive& operator<<(CArchive&, const std::vector<uint8_t>& a);
CArchive& operator<<(CArchive&, const std::vector<int8_t>& a);

CArchive& operator<<(CArchive&, const std::vector<bool>& a);
CArchive& operator<<(CArchive&, const std::vector<std::string>&);

#if MRPT_WORD_SIZE != 32  // If it's 32 bit, size_t <=> uint32_t
CArchive& operator<<(CArchive&, const std::vector<size_t>& a);
#endif

// Read --------------------
CArchive& operator>>(CArchive& in, char* a);
CArchive& operator>>(CArchive& in, std::string& str);

CArchive& operator>>(CArchive& in, std::vector<int32_t>& a);
CArchive& operator>>(CArchive& in, std::vector<uint32_t>& a);
CArchive& operator>>(CArchive& in, std::vector<uint16_t>& a);
CArchive& operator>>(CArchive& in, std::vector<int16_t>& a);
CArchive& operator>>(CArchive& in, std::vector<int64_t>& a);
CArchive& operator>>(CArchive& in, std::vector<uint8_t>& a);
CArchive& operator>>(CArchive& in, std::vector<int8_t>& a);
CArchive& operator>>(CArchive& in, std::vector<bool>& a);

CArchive& operator>>(CArchive& in, std::vector<std::string>& a);

// For backward compatibility, since in MRPT<0.8.1 vector_XXX and
// std::vector<XXX> were exactly equivalent, now there're not.
CArchive& operator>>(CArchive& s, std::vector<float>& a);
CArchive& operator>>(CArchive& s, std::vector<double>& a);
CArchive& operator<<(CArchive& s, const std::vector<float>& a);
CArchive& operator<<(CArchive& s, const std::vector<double>& a);

#if MRPT_WORD_SIZE != 32  // If it's 32 bit, size_t <=> uint32_t
CArchive& operator>>(CArchive& s, std::vector<size_t>& a);
#endif
//

template <
	typename T, std::enable_if_t<std::is_base_of<
					mrpt::serialization::CSerializable, T>::value>* = nullptr>
CArchive& operator>>(CArchive& in, typename std::shared_ptr<T>& pObj)
{
	pObj = in.ReadObject<T>();
	return in;
}

template <typename... T>
CArchive& operator>>(CArchive& in, typename mrpt::rtti::variant<T...>& pObj)
{
	pObj = in.ReadVariant<T...>();
	return in;
}

template <typename... T>
CArchive& operator<<(
	CArchive& out, const typename mrpt::rtti::variant<T...>& pObj)
{
	pObj.match([&](auto& t) { out << t; });
	return out;
}

/** CArchive for mrpt::io::CStream classes (use as template argument).
 * \sa Easier to use via function archiveFrom() */
template <class STREAM>
class CArchiveStreamBase : public CArchive
{
	STREAM& m_s;

   public:
	CArchiveStreamBase(STREAM& s) : m_s(s) {}

   protected:
	size_t write(const void* d, size_t n) override { return m_s.Write(d, n); }
	size_t read(void* d, size_t n) override { return m_s.Read(d, n); }
};

/** Helper function to create a templatized wrapper CArchive object for a:
 * MRPT's `CStream`, `std::istream`, `std::ostream`, `std::stringstream` */
template <class STREAM>
CArchiveStreamBase<STREAM> archiveFrom(STREAM& s)
{
	return CArchiveStreamBase<STREAM>(s);
}

}  // namespace serialization
}  // namespace mrpt
