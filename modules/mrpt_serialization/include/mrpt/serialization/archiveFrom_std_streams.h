/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
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
  size_t write([[maybe_unused]] const void* d, [[maybe_unused]] size_t n) override
  {
    throw std::runtime_error("CArchiveStreamBase<std::istream>: cannot write to an input stream.");
  }
  size_t read(void* d, size_t n) override
  {
    if (m_s.read(reinterpret_cast<char*>(d), static_cast<std::streamsize>(n)))
    {
      return n;
    }
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
    if (m_s.write(reinterpret_cast<const char*>(d), static_cast<std::streamsize>(n)))
    {
      return n;
    }
    return 0;
  }
  size_t read([[maybe_unused]] void* d, [[maybe_unused]] size_t n) override
  {
    throw std::runtime_error("CArchiveStreamBase<std::ostream>: cannot read from output stream.");
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
    if (m_s.write(reinterpret_cast<const char*>(d), static_cast<std::streamsize>(n)))
    {
      return n;
    }

    return 0;
  }
  size_t read(void* d, size_t n) override
  {
    if (m_s.read(reinterpret_cast<char*>(d), static_cast<std::streamsize>(n)))
    {
      return n;
    }
    return 0;
  }
};

}  // namespace mrpt::serialization
