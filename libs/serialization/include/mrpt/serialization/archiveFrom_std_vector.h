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
#include <vector>
#include <cstdint>
#include <cstring>

namespace mrpt::serialization
{
/** Specialization for std::vector, suitable for read/write, using the
 * `vector<uint8_t>` as the underlaying stream container. Writing always happen
 * at the end of the vector. Reading starts at the beggining upon construction
 * of this wrapper class (via `archiveFrom()`).
 */
template <>
class CArchiveStreamBase<std::vector<uint8_t>> : public CArchive
{
	std::vector<uint8_t>& m_v;
	int m_pos_read{0};

   public:
	CArchiveStreamBase(std::vector<uint8_t>& v) : m_v(v) {}

   protected:
	size_t write(const void* d, size_t n) override
	{
		const size_t idx = m_v.size();
		m_v.resize(idx + n);
		::memcpy(&m_v[idx], d, n);
		return n;
	}
	size_t read(void* d, size_t n) override
	{
		const int avail = static_cast<int>(m_v.size()) - m_pos_read;
		if (avail < static_cast<int>(n))
			throw std::runtime_error(
				"CArchiveStreamBase: EOF reading from std::vector!");
		::memcpy(d, &m_v[m_pos_read], n);
		m_pos_read += n;
		return n;
	};
};
/** Read-only version of the wrapper. See archiveFrom() */
template <>
class CArchiveStreamBase<const std::vector<uint8_t>> : public CArchive
{
	const std::vector<uint8_t>& m_v;
	int m_pos_read{0};

   public:
	CArchiveStreamBase(const std::vector<uint8_t>& v) : m_v(v) {}

   protected:
	size_t write(const void* d, size_t n) override
	{
		throw std::runtime_error(
			"CArchiveStreamBase: Attempt to write to read-only vector.");
	}
	size_t read(void* d, size_t n) override
	{
		const int avail = static_cast<int>(m_v.size()) - m_pos_read;
		if (avail < static_cast<int>(n))
			throw std::runtime_error(
				"CArchiveStreamBase: EOF reading from std::vector!");
		::memcpy(d, &m_v[m_pos_read], n);
		m_pos_read += n;
		return n;
	};
};
}  // namespace mrpt::serialization
