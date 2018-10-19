/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/common.h>  // MRPT_printf_format_check
#include <cstdint>
#include <string>

namespace mrpt::io
{
/** This base class is used to provide a unified interface to
 *    files,memory buffers,..Please see the derived classes. This class is
 *    largely inspired by Borland VCL "TStream" class. <br><br>
 *  Apart of the "VCL like" methods, operators ">>" and "<<" have been
 *    defined so that simple types (int,bool,char,float,char *,std::string,...)
 *    can be directly written and read to and from any CStream easily.
 *  Please, it is recomendable to read CSerializable documentation also.
 *
 * \ingroup mrpt_io_grp
 * \sa CFileStream, CMemoryStream,CSerializable
 */
class CStream
{
   public:
	/** Used in CStream::Seek */
	enum TSeekOrigin
	{
		sFromBeginning = 0,
		sFromCurrent = 1,
		sFromEnd = 2
	};

	/** Introduces a pure virtual method responsible for reading from the
	 * stream. */
	virtual size_t Read(void* Buffer, size_t Count) = 0;

	/** Introduces a pure virtual method responsible for writing to the stream.
	 *  Write attempts to write up to Count bytes to Buffer, and returns the
	 * number of bytes actually written. */
	virtual size_t Write(const void* Buffer, size_t Count) = 0;

	/* Constructor
	 */
	CStream() = default;
	/* Destructor
	 */
	virtual ~CStream();

	/** Reads a block of bytes from the stream into Buffer, and returns the
	 *amound of bytes actually read, without waiting for more extra bytes to
	 *arrive (just those already enqued in the stream).
	 *  Note that this method will fallback to ReadBuffer() in most CStream
	 *classes but in some hardware-related  classes.
	 *	\exception std::exception On any error, or if ZERO bytes are read.
	 */
	virtual size_t ReadBufferImmediate(void* Buffer, size_t Count)
	{
		return Read(Buffer, Count);
	}

	/** Introduces a pure virtual method for moving to a specified position in
	 *the streamed resource.
	 *   he Origin parameter indicates how to interpret the Offset parameter.
	 *Origin should be one of the following values:
	 *	- sFromBeginning	(Default) Offset is from the beginning of the
	 *resource. Seek moves to the position Offset. Offset must be >= 0.
	 *	- sFromCurrent		Offset is from the current position in the resource.
	 *Seek moves to Position + Offset.
	 *	- sFromEnd			Offset is from the end of the resource. Offset must
	 *be
	 *<= 0 to indicate a number of bytes before the end of the file.
	 * \return Seek returns the new value of the Position property.
	 */
	virtual uint64_t Seek(
		int64_t Offset, CStream::TSeekOrigin Origin = sFromBeginning) = 0;

	/** Returns the total amount of bytes in the stream.
	 */
	virtual uint64_t getTotalBytesCount() const = 0;

	/** Method for getting the current cursor position, where 0 is the first
	 * byte and TotalBytesCount-1 the last one.
	 */
	virtual uint64_t getPosition() const = 0;

	/** Writes a string to the stream in a textual form.
	 * \sa CStdOutStream
	 */
	virtual int printf(const char* fmt, ...)
		MRPT_printf_format_check(2, 3);  // The first argument (1) is "this" !!!

	/** Prints a vector in the format [A,B,C,...] using CStream::printf, and the
	 * fmt string for <b>each</b> vector element `T`.
	 * \tparam CONTAINER_TYPE can be any vector<T>, deque<T> or alike. */
	template <typename CONTAINER_TYPE>
	void printf_vector(
		const char* fmt, const CONTAINER_TYPE& V, char separator = ',')
	{
		this->printf("[");
		const size_t N = V.size();
		for (size_t i = 0; i < N; i++)
		{
			this->printf(fmt, V[i]);
			if (i != (N - 1)) this->printf("%c", separator);
		}
		this->printf("]");
	}

	/** Reads from the stream until a '\n' character is found ('\r' characters
	 * are ignored).
	 * \return false on EOF or any other read error.
	 */
	bool getline(std::string& out_str);

};  // End of class def.

}  // namespace mrpt::io
