/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstdint>

/** Minimal serializable classes shared by this module's unit tests. */
namespace SerTestNS
{
/** Note the short class name: stripped of its namespace it is exactly 3 chars,
 * which keeps the hand-crafted pre-MRPT-0.5.5 stream headers in the tests
 * readable. */
class Foo : public mrpt::serialization::CSerializable
{
 public:
  uint32_t m_value{0};

  DEFINE_SERIALIZABLE(Foo, SerTestNS)
};

/** A second class, to check that class identities are enforced on reading. */
class Bar : public mrpt::serialization::CSerializable
{
 public:
  double m_value{0};

  DEFINE_SERIALIZABLE(Bar, SerTestNS)
};

/** A class that does support schema-based (JSON,...) serialization. */
class WithSchema : public mrpt::serialization::CSerializable
{
 public:
  int32_t m_int{0};
  std::string m_str;

  DEFINE_SERIALIZABLE(WithSchema, SerTestNS)
  DEFINE_SCHEMA_SERIALIZABLE()
};

enum class TestEnum : uint8_t
{
  First = 0,
  Second
};

/** Registers the classes above exactly once, whatever test runs first. */
void registerTestClasses();

}  // namespace SerTestNS

MRPT_ENUM_TYPE_BEGIN(SerTestNS::TestEnum)
MRPT_FILL_ENUM_MEMBER(SerTestNS::TestEnum, First);
MRPT_FILL_ENUM_MEMBER(SerTestNS::TestEnum, Second);
MRPT_ENUM_TYPE_END()
