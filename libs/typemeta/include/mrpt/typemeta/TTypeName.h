/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstdint>
#include <mrpt/typemeta/static_string.h>

// frwd decl for TTypeName specialization:
namespace std
{
template <class T>
class shared_ptr;
}

namespace mrpt
{
namespace typemeta
{
/** @name Conversion of type to string at compile time
  * IMPORTANT: See also <mrpt/typemeta/TTypeName_stl.h> for definitions for STL
  * containers, and <mrpt/serialization/stl_serialization.h> for serialization.
@{ */

/** A template to obtain the type of its argument as a string at compile time.
 *  It works with all classes derived from  CObject, plus many
 * specializations for the plain data types (bool, double, uint8_t, etc...)
 *   For example:
 *  \code
 *     cout << TTypeName<double>::get() << endl;  // "double"
 *   	cout << TTypeName<CPose2D>::get() << endl; // "CPose2D"
 *  \endcode
 *
 *  Users can extend this for custom structs/classes with the macro
 * DECLARE_CUSTOM_TTYPENAME:
 *  \code
 *     class MyClass { ... };
 *     DECLARE_CUSTOM_TTYPENAME(MyClass)
 *     cout << TTypeName<MyClass>::get() << endl;  // "MyClass"
 *  \endcode
 * or alternatively, to avoid adding out-of-class macros:
 *  \code
 *     namespace MyNS {
 *      class MyClass {
 *        DECLARE_TTYPENAME_CLASSNAME(MyNS::MyClass)
 *      };
 *     }
 *     cout << TTypeName<MyNS::MyClass>::get() << endl; // "MyNS::MyClass"
 *  \endcode
 *  The following types are NOT ALLOWED since they have platform-dependant
 * sizes:
 *  - int, unsigned int
 *  - long, unsigned long
 *  - short, unsigned short
 *  - size_t
 *
 * \ingroup mrpt_typemeta_grp
 */
template <typename T>
struct TTypeName
{
	constexpr static auto get() { return T::getClassName(); }
};

/** Specialization for shared_ptr<T> */
template <typename T>
struct TTypeName<std::shared_ptr<T>>
{
	constexpr static auto get()
	{
		return literal("std::shared_ptr<") + TTypeName<T>::get() + literal(">");
	}
};

/** Identical to MRPT_DECLARE_TTYPENAME but intended for user code.
 * MUST be placed at the GLOBAL namespace. Can be used for types declared
 * at the global or within some namespace. Just write the full namespace path
 * as `_TYPE` argument here.
 * \sa TTypeName, DECLARE_TTYPENAME_CLASSNAME
 */
#define DECLARE_CUSTOM_TTYPENAME(_TYPE) \
	namespace mrpt                      \
	{                                   \
	namespace typemeta                  \
	{                                   \
	MRPT_DECLARE_TTYPENAME(_TYPE)       \
	}                                   \
	}

/** Like DECLARE_CUSTOM_TTYPENAME(), but for use within the class declaration
 * body. It has the advantage of not requiring macros/definitions out of the
 * original class namespace.
 * \sa TTypeName
 */
#define DECLARE_TTYPENAME_CLASSNAME(_CLASSNAME)      \
   public:                                           \
	static constexpr auto getClassName()             \
	{                                                \
		return mrpt::typemeta::literal(#_CLASSNAME); \
	}

#define MRPT_DECLARE_TTYPENAME(_TYPE)                           \
	template <>                                                 \
	struct TTypeName<_TYPE>                                     \
	{                                                           \
		constexpr static auto get() { return literal(#_TYPE); } \
	};
/** Declares a typename to be "namespace::type"
 * \sa MRPT_DECLARE_TTYPENAME_NO_NAMESPACE */
#define MRPT_DECLARE_TTYPENAME_NAMESPACE(_TYPE, __NS)                      \
	template <>                                                            \
	struct TTypeName<__NS::_TYPE>                                          \
	{                                                                      \
		constexpr static auto get() { return literal(#__NS "::" #_TYPE); } \
	};

/** Declares a typename to be "type" (without the NS prefix)
 * \sa MRPT_DECLARE_TTYPENAME_NAMESPACE */
#define MRPT_DECLARE_TTYPENAME_NO_NAMESPACE(_TYPE, __NS)        \
	template <>                                                 \
	struct TTypeName<__NS::_TYPE>                               \
	{                                                           \
		constexpr static auto get() { return literal(#_TYPE); } \
	};

#define MRPT_DECLARE_TTYPENAME_PTR(_TYPE)                     \
	template <>                                               \
	struct TTypeName<_TYPE::Ptr>                              \
	{                                                         \
		static auto get() { return TTypeName<_TYPE>::get(); } \
	};

#define MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(_TYPE, __NS)           \
	template <>                                                     \
	struct TTypeName<__NS::_TYPE::Ptr>                              \
	{                                                               \
		static auto get() { return TTypeName<__NS::_TYPE>::get(); } \
	};

MRPT_DECLARE_TTYPENAME(bool)
MRPT_DECLARE_TTYPENAME(double)
MRPT_DECLARE_TTYPENAME(float)
MRPT_DECLARE_TTYPENAME(uint64_t)
MRPT_DECLARE_TTYPENAME(int64_t)
MRPT_DECLARE_TTYPENAME(uint32_t)
MRPT_DECLARE_TTYPENAME(int32_t)
MRPT_DECLARE_TTYPENAME(uint16_t)
MRPT_DECLARE_TTYPENAME(int16_t)
MRPT_DECLARE_TTYPENAME(uint8_t)
MRPT_DECLARE_TTYPENAME(int8_t)

/** @} */

}  // namespace typemeta
}  // namespace mrpt
