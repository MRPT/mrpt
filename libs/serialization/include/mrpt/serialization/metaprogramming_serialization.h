/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/serialization_frwds.h>

namespace mrpt::serialization::metaprogramming
{
/** \addtogroup stlext_grp
 * @{ */

/** An object for reading objects from a stream, intended for being used in STL
 * algorithms. */
struct ObjectReadFromStream
{
   private:
	mrpt::serialization::CArchive* m_stream;

   public:
	inline ObjectReadFromStream(mrpt::serialization::CArchive* stream)
		: m_stream(stream)
	{
	}
	// T can be CSerializable::Ptr, CSerializable, or any other class
	// implementing ">>"
	template <typename T>
	inline void operator()(T& obj)
	{
		(*m_stream) >> obj;
	}
};

template <typename ptr_t>
struct ObjectReadFromStreamToPtrs
{
   private:
	mrpt::serialization::CArchive* m_stream;

   public:
	inline ObjectReadFromStreamToPtrs(mrpt::serialization::CArchive* stream)
		: m_stream(stream)
	{
	}
	template <typename ptr2ptr_t>
	inline void operator()(ptr2ptr_t& obj)
	{
		ptr_t p;
		(*m_stream) >> p;
		obj = p;
	}
};

/** An object for writing objects to a stream, intended for being used in STL
 * algorithms. */
struct ObjectWriteToStream
{
   private:
	mrpt::serialization::CArchive* m_stream;

   public:
	inline ObjectWriteToStream(mrpt::serialization::CArchive* stream)
		: m_stream(stream)
	{
	}

	// T can be CSerializable::Ptr, CSerializable, or any other class
	// implementing "<<"
	template <typename T>
	inline void operator()(const T& ptr)
	{
		(*m_stream) << ptr;
	}
};

/** @} */  // end of grouping
}  // namespace mrpt::serialization::metaprogramming
