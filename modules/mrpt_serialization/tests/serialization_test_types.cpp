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

#include "serialization_test_types.h"

#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>

using namespace mrpt::serialization;

IMPLEMENTS_SERIALIZABLE(Foo, CSerializable, SerTestNS)
IMPLEMENTS_SERIALIZABLE(Bar, CSerializable, SerTestNS)
IMPLEMENTS_SERIALIZABLE(WithSchema, CSerializable, SerTestNS)

uint8_t SerTestNS::Foo::serializeGetVersion() const { return 0; }
void SerTestNS::Foo::serializeTo(CArchive& out) const { out << m_value; }
void SerTestNS::Foo::serializeFrom(CArchive& in, uint8_t version)
{
  ASSERT_EQUAL_(version, 0U);
  in >> m_value;
}

uint8_t SerTestNS::Bar::serializeGetVersion() const { return 0; }
void SerTestNS::Bar::serializeTo(CArchive& out) const { out << m_value; }
void SerTestNS::Bar::serializeFrom(CArchive& in, uint8_t version)
{
  ASSERT_EQUAL_(version, 0U);
  in >> m_value;
}

uint8_t SerTestNS::WithSchema::serializeGetVersion() const { return 1; }
void SerTestNS::WithSchema::serializeTo(CArchive& out) const { out << m_int << m_str; }
void SerTestNS::WithSchema::serializeFrom(CArchive& in, uint8_t version)
{
  ASSERT_EQUAL_(version, 1U);
  in >> m_int >> m_str;
}

void SerTestNS::WithSchema::serializeTo(CSchemeArchiveBase& out) const
{
  SCHEMA_SERIALIZE_DATATYPE_VERSION(1);
  out["int"] = m_int;
  out["str"] = m_str;
}
void SerTestNS::WithSchema::serializeFrom(CSchemeArchiveBase& in)
{
  int version = 0;
  SCHEMA_DESERIALIZE_DATATYPE_VERSION();
  ASSERT_EQUAL_(version, 1);
  m_int = static_cast<int32_t>(in["int"]);
  m_str = static_cast<std::string>(in["str"]);
}

void SerTestNS::registerTestClasses()
{
  static const bool done = []()
  {
    mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(Foo, SerTestNS));
    mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(Bar, SerTestNS));
    mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(WithSchema, SerTestNS));
    return true;
  }();
  (void)done;
}
