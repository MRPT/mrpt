/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/containers/deepcopy_ptr.h>
#include <mrpt/core/common.h>

namespace
{
class DummyClass
{
 public:
  double x = 0;

  bool operator==(const DummyClass &o) const { return x == o.x; }

  DummyClass *clone() const { return new DummyClass(*this); }
};
}  // namespace

using namespace mrpt;
using namespace std;

TEST(copy_ptr, SimpleOps)
{
  mrpt::containers::copy_ptr<int> ptr1;
  EXPECT_FALSE(ptr1);

  ptr1.reset(new int());
  EXPECT_TRUE(ptr1);

  *ptr1 = 123;
  EXPECT_TRUE(*ptr1 == 123);

  {
    mrpt::containers::copy_ptr<int> ptr2 = ptr1;
    EXPECT_TRUE(*ptr1 == *ptr2);

    (*ptr2)++;
    EXPECT_FALSE(*ptr1 == *ptr2);
  }
  {
    mrpt::containers::copy_ptr<int> ptr2;
    ptr2 = ptr1;
    EXPECT_TRUE(*ptr1 == *ptr2);

    (*ptr2)++;
    EXPECT_FALSE(*ptr1 == *ptr2);
  }
}

TEST(copy_ptr, StlContainer)
{
  using str2d_ptr = mrpt::containers::copy_ptr<std::pair<std::string, std::size_t>>;

  str2d_ptr ptr;
  EXPECT_FALSE(ptr);

  std::vector<str2d_ptr> v;
  for (std::size_t i = 0; i < 10; i++)
  {
    v.push_back(str2d_ptr(new str2d_ptr::value_type));
    v[i]->first = "xxx";
    v[i]->second = i;
  }

  str2d_ptr v3 = v[3];
  EXPECT_TRUE(v3->second == 3);
  v3->second++;

  EXPECT_TRUE(v3->second == 4);
  EXPECT_TRUE(v[3]->second == 3);
}

TEST(poly_ptr, SimpleOps)
{
  mrpt::containers::poly_ptr<DummyClass> ptr1;
  EXPECT_FALSE(ptr1);

  ptr1.reset(new DummyClass());
  EXPECT_TRUE(ptr1);

  ptr1->x = 123.0;
  EXPECT_NEAR(ptr1->x, 123.0, 1e-9);

  {
    mrpt::containers::poly_ptr<DummyClass> ptr2 = ptr1;
    EXPECT_TRUE(*ptr1 == *ptr2);

    ptr2->x += 1.0;
    EXPECT_FALSE(*ptr1 == *ptr2);
  }
  {
    mrpt::containers::poly_ptr<DummyClass> ptr2;
    ptr2 = ptr1;
    EXPECT_TRUE(*ptr1 == *ptr2);

    ptr2->x += 1.0;
    EXPECT_FALSE(*ptr1 == *ptr2);
  }
}

TEST(poly_ptr, StlContainer)
{
  using str2d_ptr = mrpt::containers::poly_ptr<DummyClass>;

  str2d_ptr ptr;
  EXPECT_FALSE(ptr);

  std::vector<str2d_ptr> v;
  for (std::size_t i = 0; i < 10; i++)
  {
    v.push_back(str2d_ptr(new str2d_ptr::value_type));
    v[i]->x = static_cast<double>(i);
  }

  str2d_ptr v3 = v[3];
  EXPECT_NEAR(v3->x, 3.0, 1e-9);
  v3->x += 1.0;

  EXPECT_NEAR(v3->x, 4.0, 1e-9);
  EXPECT_NEAR(v[3]->x, 3.0, 1e-9);
}
