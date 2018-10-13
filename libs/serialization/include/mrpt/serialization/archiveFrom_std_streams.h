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
#include <iosfwd>
#include <istream>
#include <ostream>

namespace mrpt::serialization
{
/** Specialization for std::istream */
template <>
class CArchiveStreamBase<std::istream> : public CArchive
{
	std::istream& m_s;

   public:
	CArchiveStreamBase(std::istream& s) : m_s(s) {}

   protected:
	size_t write(const void* d, size_t n) override
	{
		throw std::runtime_error(
			"CArchiveStreamBase<std::istream>:"
			"cannot write to an input stream.");
	}
	size_t read(void* d, size_t n) override
	{
		if (m_s.read(reinterpret_cast<char*>(d), n))
			return n;
		else
			return 0;
	}
};

/** Specialization for std::ostream */
template <>
class CArchiveStreamBase<std::ostream> : public CArchive
{
	std::ostream& m_s;

   public:
	CArchiveStreamBase(std::ostream& s) : m_s(s) {}

   protected:
	size_t write(const void* d, size_t n) override
	{
		if (m_s.write(reinterpret_cast<const char*>(d), n))
			return n;
		else
			return 0;
	}
	size_t read(void* d, size_t n) override
	{
		throw std::runtime_error(
			"CArchiveStreamBase<std::ostream>:"
			"cannot read from output stream.");
	}
};

/** Specialization for std::iostream */
template <>
class CArchiveStreamBase<std::iostream> : public CArchive
{
	std::iostream& m_s;

   public:
	CArchiveStreamBase(std::iostream& s) : m_s(s) {}

   protected:
	size_t write(const void* d, size_t n) override
	{
		if (m_s.write(reinterpret_cast<const char*>(d), n))
			return n;
		else
			return 0;
	}
	size_t read(void* d, size_t n) override
	{
		if (m_s.read(reinterpret_cast<char*>(d), n))
			return n;
		else
			return 0;
	}
};

}  // namespace mrpt::serialization
