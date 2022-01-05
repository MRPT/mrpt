/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/poses/CPoseInterpolatorBase.h>  // to test with an enum
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/optional_serialization.h>

using namespace mrpt::serialization;

namespace MyNS
{
class Foo : public CSerializable
{
	DEFINE_SERIALIZABLE(Foo, MyNS)
   public:
	Foo() = default;
	Foo(uint16_t v) : value(v) {}

	bool operator==(const Foo& o) const { return o.value == value; }

	int16_t value = 0;
};
}  // namespace MyNS

IMPLEMENTS_SERIALIZABLE(Foo, CSerializable, MyNS);

uint8_t MyNS::Foo::serializeGetVersion() const { return 0; }
void MyNS::Foo::serializeTo(CArchive& out) const { out << value; }
void MyNS::Foo::serializeFrom(CArchive& in, uint8_t serial_version)
{
	in >> value;
}

TEST(Serialization, CustomClassSerialize)
{
	mrpt::rtti::registerClass(CLASS_ID(MyNS::Foo));

	MyNS::Foo a;
	a.value = 123;

	mrpt::io::CMemoryStream buf;
	auto arch = mrpt::serialization::archiveFrom(buf);
	arch << a;

	buf.Seek(0);
	MyNS::Foo b;
	arch >> b;

	EXPECT_EQ(a.value, b.value);
}

TEST(Serialization, ArchiveSharedPtrs)
{
	mrpt::io::CMemoryStream buf;
	auto arch_ptr = mrpt::serialization::archivePtrFrom(buf);
	auto arch_ptr2 = mrpt::serialization::archiveUniquePtrFrom(buf);

	int a = 42;
	(*arch_ptr) << a;
	buf.Seek(0);

	int b;
	(*arch_ptr2) >> b;

	EXPECT_EQ(a, b);
}

TEST(Serialization, optionalObjects)
{
	mrpt::rtti::registerClass(CLASS_ID(MyNS::Foo));

	mrpt::io::CMemoryStream buf;
	auto arch = mrpt::serialization::archiveFrom(buf);

	std::optional<int> a = 42, b;
	std::optional<MyNS::Foo> c(123), d;

	arch << a << b << c << d;
	buf.Seek(0);

	std::optional<int> a2, b2;
	std::optional<MyNS::Foo> c2, d2;
	arch >> a2 >> b2 >> c2 >> d2;

	EXPECT_TRUE(a2.has_value());
	EXPECT_EQ(*a, *a2);

	EXPECT_TRUE(c2.has_value());
	EXPECT_EQ(*c, *c2);

	EXPECT_FALSE(b2.has_value());
	EXPECT_FALSE(d2.has_value());

	EXPECT_EQ(a, a2);
	EXPECT_EQ(b, b2);
	EXPECT_EQ(c, c2);
	EXPECT_EQ(d, d2);
}

TEST(Serialization, enums)
{
	mrpt::io::CMemoryStream buf;
	auto arch = mrpt::serialization::archiveFrom(buf);

	const mrpt::poses::TInterpolatorMethod im1 = mrpt::poses::imSpline;
	arch << im1;

	buf.Seek(0);
	mrpt::poses::TInterpolatorMethod im2;
	arch >> im2;

	EXPECT_EQ(im1, im2);
}
