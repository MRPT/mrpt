// /* +------------------------------------------------------------------------+
//    |                     Mobile Robot Programming Toolkit (MRPT)            |
//    |                          http://www.mrpt.org/                          |
//    |                                                                        |
//    | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
//    | See: http://www.mrpt.org/Authors - All rights reserved.                |
//    | Released under BSD License. See details in http://www.mrpt.org/License |
//    +------------------------------------------------------------------------+ */

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSchemeArchiveBase.h>
#include <mrpt/io/CMemoryStream.h>
#include <gtest/gtest.h>

using namespace mrpt::serialization;
namespace TestNS
{   
    template <typename SCHEME_CAPABLE>
    class MockSchemeArchive : public CSchemeArchiveBase_impl
    {
		public:
        MockSchemeArchive(SCHEME_CAPABLE& val):m_val(val) {}
        //Virtual assignment operators
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const int32_t val) override
        {
            m_val = val;
            return *m_parent;
        }
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const uint32_t val) override
        {
            m_val = val;
            return *m_parent;
        }
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const int64_t val) override
        {
            m_val = val;
            return *m_parent;
        }
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const uint64_t val) override
        {
            m_val = val;
            return *m_parent;
        }
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const float val) override
        {
            m_val = val;
            return *m_parent;
        }
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const double val) override
        {
            m_val = val;
            return *m_parent;
        }
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const std::nullptr_t val) override
        {
            // m_val = val;
            return *m_parent;
        }
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const std::string val) override
        {
            // m_val = val;
            return *m_parent;
        }
        virtual mrpt::serialization::CSchemeArchiveBase &operator=(const bool val) override
        {
            m_val = val;
            return *m_parent;
        }

        virtual explicit operator int32_t() const override
        {
            return 1;
        }
        virtual explicit operator uint32_t() const override
        {
            return 1;
        }
        virtual explicit operator int64_t() const override
        {
            return 1;
        }
        virtual explicit operator uint64_t() const override
        {
            return 1;
        }
        virtual explicit operator float() const override
        {
            return 1.0;
        }
        virtual explicit operator double() const override
        {
            return 1.0;
        }
        virtual explicit operator bool() const override
        {
            return 1;
        }
        virtual explicit operator std::string() const override
        {
            return "str";
        }

        virtual mrpt::serialization::CSchemeArchiveBase &operator=(mrpt::serialization::CSerializable& obj) override
        {
            ReadObject(*m_parent, obj);
            return *m_parent;
        }
        virtual void asSerializableObject(mrpt::serialization::CSerializable& obj) override
        {
            WriteObject(*m_parent, obj);
            return;
        }

        virtual CSchemeArchiveBase operator[](size_t idx) override
        {
            return CSchemeArchiveBase(std::make_unique<MockSchemeArchive<SCHEME_CAPABLE>>(m_val));
        }
        virtual CSchemeArchiveBase operator[](std::string str) override
        {  
            return CSchemeArchiveBase(std::make_unique<MockSchemeArchive<SCHEME_CAPABLE>>(m_val));
        }

        private:
            SCHEME_CAPABLE& m_val;
    };
    class Bar : public mrpt::serialization::CSerializable
    {
        DEFINE_SERIALIZABLE(Bar)
        DEFINE_SCHEMA_SERIALIZABLE()
        public:
            int32_t value;
    };
}
IMPLEMENTS_SERIALIZABLE(Bar, CSerializable, TestNS)

uint8_t TestNS::Bar::serializeGetVersion() const { return 0; }
void TestNS::Bar::serializeTo(CArchive& out) const { out << value; }
void TestNS::Bar::serializeFrom(CArchive& in, uint8_t serial_version)
{
	in >> value;
}
void TestNS::Bar::serializeFrom(CSchemeArchiveBase& in)
{
    value = static_cast<int32_t>(in);
}
void TestNS::Bar::serializeTo(CSchemeArchiveBase& out) const
{
    out = value;
}



TEST(SchemaSerialization, CustomClassSchemeSerialize)
{
    int t;
    CSchemeArchiveBase wrapper(std::make_unique<TestNS::MockSchemeArchive<int>>(t));
    wrapper["foo"][1] = 1;
    wrapper["bar"][3] = 3;
    EXPECT_EQ(1,1);    
}