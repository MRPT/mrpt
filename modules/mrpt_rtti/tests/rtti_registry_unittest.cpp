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

#include <gtest/gtest.h>
#include <mrpt/rtti/CListOfClasses.h>
#include <mrpt/rtti/CObject.h>

#include <algorithm>
#include <string>

namespace RegNS
{
/** A virtual (non-instantiable) base, to exercise the DEFINE_VIRTUAL_MRPT_OBJECT
 * path and the "no dynamic constructor" branch of createObject(). */
class VirtBase : public mrpt::rtti::CObject
{
  DEFINE_VIRTUAL_MRPT_OBJECT(VirtBase, RegNS)
};

class Leaf : public VirtBase
{
 public:
  int m_value{0};

  Leaf() = default;
  Leaf(int v) : m_value(v) {}
  DEFINE_MRPT_OBJECT(Leaf, RegNS)
};

/** A class without a copy constructor, to exercise CopyCtor<false>::clone(). */
class NonCopyable : public mrpt::rtti::CObject
{
 public:
  NonCopyable() = default;
  ~NonCopyable() override = default;
  NonCopyable(const NonCopyable&) = delete;
  NonCopyable& operator=(const NonCopyable&) = delete;
  NonCopyable(NonCopyable&&) = default;
  NonCopyable& operator=(NonCopyable&&) = default;
  DEFINE_MRPT_OBJECT(NonCopyable, RegNS)
};

/** A bare CObject descendant, i.e. without the DEFINE_MRPT_OBJECT machinery. */
class PlainObject : public mrpt::rtti::CObject
{
 public:
  [[nodiscard]] mrpt::rtti::CObject* clone() const override { return new PlainObject(*this); }
};

}  // namespace RegNS

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(VirtBase, mrpt::rtti::CObject, RegNS)
IMPLEMENTS_MRPT_OBJECT(Leaf, RegNS::VirtBase, RegNS)
IMPLEMENTS_MRPT_OBJECT(NonCopyable, mrpt::rtti::CObject, RegNS)

namespace
{
/** Registers this file's classes exactly once, whatever test runs first. */
void registerTestClasses()
{
  static const bool done = []()
  {
    mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(VirtBase, RegNS));
    mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(Leaf, RegNS));
    mrpt::rtti::registerClass(CLASS_ID_NAMESPACE(NonCopyable, RegNS));
    return true;
  }();
  (void)done;
}

bool listContains(
    const std::vector<const mrpt::rtti::TRuntimeClassId*>& lst, const std::string& className)
{
  return std::any_of(
      lst.begin(), lst.end(),
      [&](const mrpt::rtti::TRuntimeClassId* id) { return className == id->className; });
}
}  // namespace

TEST(rtti_registry, findRegisteredClass)
{
  registerTestClasses();

  const auto* id = mrpt::rtti::findRegisteredClass("RegNS::Leaf");
  ASSERT_NE(id, nullptr);
  EXPECT_EQ(id, CLASS_ID(RegNS::Leaf));

  // Unregistered names always yield nullptr:
  EXPECT_EQ(mrpt::rtti::findRegisteredClass("RegNS::NotAClass"), nullptr);
}

TEST(rtti_registry, findRegisteredClassIgnoringNamespace)
{
  registerTestClasses();

  // Datasets older than MRPT v2.0 stored class names without namespace, which
  // is why the namespace-less lookup is enabled by default:
  EXPECT_EQ(mrpt::rtti::findRegisteredClass("Leaf"), CLASS_ID(RegNS::Leaf));

  // ...and can be disabled for strictly-namespaced streams:
  EXPECT_EQ(mrpt::rtti::findRegisteredClass("Leaf", false), nullptr);

  // A fully-qualified name must not depend on the fallback either:
  EXPECT_EQ(mrpt::rtti::findRegisteredClass("RegNS::Leaf", false), CLASS_ID(RegNS::Leaf));
}

TEST(rtti_registry, registerClassCustomName)
{
  registerTestClasses();

  mrpt::rtti::registerClassCustomName("AnAliasForLeaf", CLASS_ID(RegNS::Leaf));

  EXPECT_EQ(mrpt::rtti::findRegisteredClass("AnAliasForLeaf"), CLASS_ID(RegNS::Leaf));

  auto o = mrpt::rtti::classFactory("AnAliasForLeaf");
  ASSERT_TRUE(o);
  EXPECT_TRUE(IS_CLASS(*o, RegNS::Leaf));
}

TEST(rtti_registry, getAllRegisteredClasses)
{
  registerTestClasses();

  const auto lst = mrpt::rtti::getAllRegisteredClasses();
  EXPECT_FALSE(lst.empty());
  EXPECT_TRUE(listContains(lst, "RegNS::Leaf"));
  EXPECT_TRUE(listContains(lst, "RegNS::VirtBase"));
}

TEST(rtti_registry, getAllRegisteredClassesChildrenOf)
{
  registerTestClasses();

  {
    const auto lst = mrpt::rtti::getAllRegisteredClassesChildrenOf(CLASS_ID(RegNS::VirtBase));
    EXPECT_TRUE(listContains(lst, "RegNS::Leaf"));
    // The parent itself is never part of its own children list:
    EXPECT_FALSE(listContains(lst, "RegNS::VirtBase"));
    EXPECT_FALSE(listContains(lst, "RegNS::NonCopyable"));
  }
  {
    // A leaf class has no children:
    const auto lst = mrpt::rtti::getAllRegisteredClassesChildrenOf(CLASS_ID(RegNS::Leaf));
    EXPECT_FALSE(listContains(lst, "RegNS::Leaf"));
  }
}

TEST(rtti_registry, classFactoryUnknownName)
{
  EXPECT_EQ(mrpt::rtti::classFactory("RegNS::ThisDoesNotExist"), nullptr);
}

TEST(rtti_registry, createObjectOfVirtualClass)
{
  registerTestClasses();

  // A DEFINE_VIRTUAL_MRPT_OBJECT class has no ptrCreateObject: creating it
  // must fail gracefully with an empty pointer, not crash.
  EXPECT_FALSE(CLASS_ID(RegNS::VirtBase)->createObject());
  EXPECT_EQ(mrpt::rtti::classFactory("RegNS::VirtBase"), nullptr);
}

TEST(rtti_registry, derivedFromByName)
{
  registerTestClasses();

  const auto* leaf = CLASS_ID(RegNS::Leaf);

  EXPECT_TRUE(leaf->derivedFrom("RegNS::Leaf"));  // same class
  EXPECT_TRUE(leaf->derivedFrom("RegNS::VirtBase"));
  EXPECT_FALSE(leaf->derivedFrom("RegNS::NonCopyable"));

  // A class not derived from the given one, walking up to the root:
  EXPECT_FALSE(CLASS_ID(mrpt::rtti::CObject)->derivedFrom("RegNS::Leaf"));

  // Unregistered base names are a programming error. Note that the root class
  // mrpt::rtti::CObject is itself never registered, so it cannot be used as
  // the argument of this by-name overload (use the TRuntimeClassId one):
  EXPECT_ANY_THROW(leaf->derivedFrom("RegNS::ThisDoesNotExist"));
  EXPECT_ANY_THROW(leaf->derivedFrom("mrpt::rtti::CObject"));
}

TEST(rtti_registry, derivedFromById)
{
  registerTestClasses();

  const auto* leaf = CLASS_ID(RegNS::Leaf);

  EXPECT_TRUE(leaf->derivedFrom(CLASS_ID(RegNS::Leaf)));
  EXPECT_TRUE(leaf->derivedFrom(CLASS_ID(RegNS::VirtBase)));
  EXPECT_TRUE(leaf->derivedFrom(CLASS_ID(mrpt::rtti::CObject)));
  EXPECT_FALSE(leaf->derivedFrom(CLASS_ID(RegNS::NonCopyable)));

  EXPECT_ANY_THROW(leaf->derivedFrom(static_cast<const mrpt::rtti::TRuntimeClassId*>(nullptr)));
}

TEST(rtti_registry, cloneAndDuplicateGetSmartPtr)
{
  RegNS::Leaf o(42);

  {
    auto p = std::unique_ptr<mrpt::rtti::CObject>(o.clone());
    ASSERT_TRUE(p);
    EXPECT_TRUE(IS_CLASS(*p, RegNS::Leaf));
    EXPECT_EQ(dynamic_cast<RegNS::Leaf&>(*p).m_value, 42);
  }
  {
    auto p = o.duplicateGetSmartPtr();
    ASSERT_TRUE(p);
    EXPECT_EQ(std::dynamic_pointer_cast<RegNS::Leaf>(p)->m_value, 42);
  }
}

TEST(rtti_registry, cloneNonCopyableThrows)
{
  RegNS::NonCopyable o;
  EXPECT_ANY_THROW(o.clone());
}

TEST(rtti_registry, ptrCastFromBaseToDerived)
{
  mrpt::rtti::CObject::Ptr base = RegNS::Leaf::Create(7);

  auto derived = mrpt::ptr_cast<RegNS::Leaf>::from(base);
  ASSERT_TRUE(derived);
  EXPECT_EQ(derived->m_value, 7);

  // A cast to an unrelated class yields an empty pointer:
  EXPECT_FALSE(mrpt::ptr_cast<RegNS::NonCopyable>::from(base));
}

TEST(rtti_registry, getClassNameAndClassNameMember)
{
  EXPECT_EQ(std::string(RegNS::Leaf::className), "RegNS::Leaf");
  EXPECT_EQ(std::string(RegNS::Leaf::getClassName().c_str()), "RegNS::Leaf");
}

TEST(rtti_registry, listOfClassesMultipleEntries)
{
  registerTestClasses();

  mrpt::rtti::CListOfClasses l;
  l.insert(CLASS_ID(RegNS::Leaf));
  l.insert(CLASS_ID(RegNS::NonCopyable));

  // asString() is ordered by the underlying std::set (pointer order), so only
  // check that both names show up with the documented ", " separator:
  const std::string s = l.asString();
  EXPECT_NE(s.find("RegNS::Leaf"), std::string::npos);
  EXPECT_NE(s.find("RegNS::NonCopyable"), std::string::npos);
  EXPECT_NE(s.find(", "), std::string::npos);

  // ...and that it round-trips through fromString():
  mrpt::rtti::CListOfClasses l2;
  l2.fromString(s);
  EXPECT_EQ(l2.data, l.data);
}

TEST(rtti_registry, listOfClassesFromStringTrimsBlanks)
{
  registerTestClasses();

  mrpt::rtti::CListOfClasses l;
  l.fromString("  RegNS::Leaf ,\tRegNS::NonCopyable  ");

  EXPECT_EQ(l.data.size(), 2U);
  EXPECT_TRUE(l.contains(CLASS_ID(RegNS::Leaf)));
  EXPECT_TRUE(l.contains(CLASS_ID(RegNS::NonCopyable)));
}

TEST(rtti_registry, listOfClassesFromStringClearsPreviousContents)
{
  registerTestClasses();

  mrpt::rtti::CListOfClasses l;
  l.insert(CLASS_ID(RegNS::NonCopyable));
  l.fromString("RegNS::Leaf");

  EXPECT_EQ(l.data.size(), 1U);
  EXPECT_TRUE(l.contains(CLASS_ID(RegNS::Leaf)));
  EXPECT_FALSE(l.contains(CLASS_ID(RegNS::NonCopyable)));
}

TEST(rtti_registry, listOfClassesEmptyAsString)
{
  mrpt::rtti::CListOfClasses l;
  EXPECT_EQ(l.asString(), "");
}

TEST(rtti_registry, listOfClassesContainsDerivedFrom)
{
  registerTestClasses();

  mrpt::rtti::CListOfClasses l;
  l.insert(CLASS_ID(RegNS::Leaf));

  EXPECT_TRUE(l.containsDerivedFrom(CLASS_ID(RegNS::VirtBase)));
  EXPECT_FALSE(l.containsDerivedFrom(CLASS_ID(RegNS::NonCopyable)));
}

TEST(rtti_registry, registerClassWithNullArguments)
{
  // Both are misuses that must warn instead of crashing:
  EXPECT_NO_THROW(mrpt::rtti::registerClass(nullptr));

  const mrpt::rtti::TRuntimeClassId noName{};
  EXPECT_NO_THROW(mrpt::rtti::registerClass(&noName));
}

TEST(rtti_registry, registerCustomNameTwiceWithDifferentClasses)
{
  registerTestClasses();

  // The second registration warns and wins:
  mrpt::rtti::registerClassCustomName("ADuplicatedAlias", CLASS_ID(RegNS::Leaf));
  mrpt::rtti::registerClassCustomName("ADuplicatedAlias", CLASS_ID(RegNS::NonCopyable));

  EXPECT_EQ(mrpt::rtti::findRegisteredClass("ADuplicatedAlias"), CLASS_ID(RegNS::NonCopyable));

  // Re-registering the very same class is not a conflict:
  EXPECT_NO_THROW(
      mrpt::rtti::registerClassCustomName("ADuplicatedAlias", CLASS_ID(RegNS::NonCopyable)));
}

TEST(rtti_registry, plainCObjectDescendantRuntimeClass)
{
  // A class deriving from CObject without DEFINE_MRPT_OBJECT inherits the base
  // implementation of GetRuntimeClass():
  RegNS::PlainObject o;
  EXPECT_EQ(o.GetRuntimeClass(), CLASS_ID(mrpt::rtti::CObject));
  EXPECT_TRUE(IS_CLASS(o, mrpt::rtti::CObject));

  auto p = o.duplicateGetSmartPtr();
  ASSERT_TRUE(p);
  EXPECT_EQ(p->GetRuntimeClass(), CLASS_ID(mrpt::rtti::CObject));
}
