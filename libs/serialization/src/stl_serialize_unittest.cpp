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
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/serialization/optional_serialization.h>
#include <mrpt/serialization/stl_serialization.h>

#include <memory>  // shared_ptr

using namespace mrpt::serialization;

TEST(Serialization, STL_stdvector)
{
	std::vector<uint32_t> m2, m1{1, 2, 3};

	mrpt::io::CMemoryStream f;
	auto arch = mrpt::serialization::archiveFrom(f);
	arch << m1;

	f.Seek(0);
	arch >> m2;
	EXPECT_EQ(m1, m2);
}

TEST(Serialization, STL_stdmap)
{
	std::map<uint32_t, uint8_t> m2, m1;

	m1[2] = 21;
	m1[9] = 91;

	mrpt::io::CMemoryStream f;
	auto arch = mrpt::serialization::archiveFrom(f);
	arch << m1;

	f.Seek(0);
	arch >> m2;
	EXPECT_EQ(m1, m2);
}

TEST(Serialization, STL_complex_error_type)
{
	std::map<double, std::array<uint8_t, 2>> v1;
	std::map<double, std::array<int8_t, 2>> v2;	 // different type!

	v1[0.4].fill(2);

	mrpt::io::CMemoryStream f;
	auto arch = mrpt::serialization::archiveFrom(f);
	arch << v1;

	// Trying to read to a different variable raises an exception:
	f.Seek(0);
	EXPECT_THROW(arch >> v2, std::exception);
}

TEST(Serialization, optional_STL)
{
	using map2array_t = std::map<double, std::array<uint8_t, 2>>;

	std::optional<map2array_t> v1a, v1b;
	std::optional<map2array_t> v2a, v2b;

	// v1a: leave empty.
	// v1b: assign value
	v1b.emplace();
	(*v1b)[0.4].fill(2);

	mrpt::io::CMemoryStream f;
	auto arch = mrpt::serialization::archiveFrom(f);
	arch << v1a << v1b;

	f.Seek(0);
	arch >> v2a >> v2b;
	EXPECT_FALSE(v2a.has_value());
	EXPECT_EQ(*v1b, *v2b);
}

struct Foo
{
	Foo(int i = 0) : m_i(i) {}
	int m_i;

	DECLARE_TTYPENAME_CLASSNAME(Foo)
	bool operator==(const Foo& b) const { return m_i == b.m_i; }
};
CArchive& operator<<(CArchive& a, const Foo& f)
{
	a << f.m_i;
	return a;
}
CArchive& operator>>(CArchive& a, Foo& f)
{
	a >> f.m_i;
	return a;
}

TEST(Serialization, vector_custom_type)
{
	std::vector<Foo> m2, m1{1, 2, 3};

	mrpt::io::CMemoryStream f;
	auto arch = mrpt::serialization::archiveFrom(f);
	arch << m1;

	f.Seek(0);
	arch >> m2;
	EXPECT_EQ(m1, m2);
}

TEST(Serialization, vector_shared_ptr)
{
	std::vector<std::shared_ptr<Foo>> m2, m1;
	m1.push_back(std::make_shared<Foo>(1));
	m1.push_back(std::make_shared<Foo>(2));
	m1.push_back(std::shared_ptr<Foo>());  // nullptr
	m1.push_back(std::make_shared<Foo>(3));

	mrpt::io::CMemoryStream f;
	auto arch = mrpt::serialization::archiveFrom(f);
	arch << m1;

	f.Seek(0);
	arch >> m2;
	EXPECT_EQ(m1.size(), m2.size());
	for (auto i = 0U; i < m1.size(); i++)
	{
		if (!m1[i]) { EXPECT_EQ(m1[i], m2[i]); }
		else
		{
			EXPECT_EQ(*m1[i], *m2[i]);
		}
	}
}
