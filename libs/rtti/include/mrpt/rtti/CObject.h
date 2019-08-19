/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/aligned_allocator.h>
#include <mrpt/core/safe_pointers.h>
#include <mrpt/typemeta/static_string.h>  // literal()
#include <functional>
#include <memory>
#include <vector>

namespace mrpt
{
namespace rtti
{
/** @name RTTI classes and functions for polymorphic hierarchies
	@{ */
class CObject;

/** A structure that holds runtime class type information. Use
 * CLASS_ID(<class_name>) to get a reference to the class_name's TRuntimeClassId
 * descriptor.
 * \ingroup mrpt_rtti_grp
 */
struct TRuntimeClassId
{
	using Ptr = safe_ptr<TRuntimeClassId>;
	const char* className;
	/** Create an object of the related class, or nullptr if it is virtual. */
	std::function<std::shared_ptr<CObject>(void)> ptrCreateObject;
	/** Gets the base class runtime id. */
	const TRuntimeClassId* (*getBaseClass)();

	// Operations
	std::shared_ptr<CObject> createObject() const;
	bool derivedFrom(const TRuntimeClassId* pBaseClass) const;
	bool derivedFrom(const char* pBaseClass_name) const;
};

/** Register a class into the MRPT internal list of "CObject" descendents.
 *  Used internally in the macros DEFINE_SERIALIZABLE, etc...
 * \sa getAllRegisteredClasses
 */
void registerClass(const mrpt::rtti::TRuntimeClassId* pNewClass);

/** Mostly for internal use within mrpt sources, to handle exceptional cases
 * with multiple serialization names for backward compatibility
 * (CMultiMetricMaps, CImage,...)
 */
void registerClassCustomName(
	const char* customName, const TRuntimeClassId* pNewClass);

/** Returns a list with all the classes registered in the system through
 * mrpt::rtti::registerClass.
 * \sa registerClass, findRegisteredClass
 */
std::vector<const mrpt::rtti::TRuntimeClassId*> getAllRegisteredClasses();

/** Like getAllRegisteredClasses(), but filters the list to only include
 * children clases of a given base one.
 * \sa getAllRegisteredClasses(), getAllRegisteredClassesChildrenOf()  */
std::vector<const TRuntimeClassId*> getAllRegisteredClassesChildrenOf(
	const TRuntimeClassId* parent_id);

/** Return info about a given class by its name, or nullptr if the class is not
 * registered
 * \sa registerClass, getAllRegisteredClasses
 */
const TRuntimeClassId* findRegisteredClass(const std::string& className);

template <typename T>
struct CLASS_ID_impl
{
	static constexpr const mrpt::rtti::TRuntimeClassId* get()
	{
		return &T::GetRuntimeClassIdStatic();
	}
};
//(A specialization for variant's monostate is provided in CArchive.h)

/** Access to runtime class ID for a defined class name.
 */
#define CLASS_ID(T) mrpt::rtti::CLASS_ID_impl<T>::get()
// Convert these
#define CLASS_ID_TEMPLATE(class_name, T) mrpt::rtti::CLASS_ID_impl<T>::get()
#define CLASS_ID_NAMESPACE(class_name, namespaceName) \
	mrpt::rtti::CLASS_ID_impl<namespaceName::class_name>::get()

template <typename T>
struct IS_CLASS_impl
{
	template <typename REF>
	static bool check(const REF& p)
	{
		return p.GetRuntimeClass() == CLASS_ID_impl<T>::get();
	}
};

namespace internal
{
template <bool is_copy_ctrtible>
struct CopyCtor;
template <>
struct CopyCtor<true>
{
	template <typename T>
	static T* clone(const T& o)
	{
		return new T(o);
	}
};
template <>
struct CopyCtor<false>
{
	template <typename T>
	static T* clone(const T& o)
	{
		throw std::runtime_error(
			"clone(): Attempt to call copy ctor of non copy-constructible "
			"class.");
	}
};
}  // namespace internal

/** True if the given reference to object (derived from mrpt::rtti::CObject) is
 * of the given class. */
#define IS_CLASS(obj, class_name) \
	mrpt::rtti::IS_CLASS_impl<class_name>::check(obj)

/** True if the given reference to object (derived from mrpt::rtti::CObject) is
 * an instance of the given class OR any of its derived classes. */
#define IS_DERIVED(obj, class_name) \
	((obj).GetRuntimeClass()->derivedFrom(CLASS_ID(class_name)))

/** Virtual base to provide a compiler-independent RTTI system.
 *
 * Each class named `Foo` will have associated smart pointer types:
 * - `Foo::Ptr` => `std::shared_ptr<Foo>` (the most commonly-used one)
 * - `Foo::ConstPtr` => `std::shared_ptr<const Foo>`
 * - `Foo::UniquePtr` => `std::unique_ptr<Foo>`
 * - `Foo::ConstUniquePtr` => `std::unique_ptr<const Foo>`
 *
 * It is recommended to use MRPT-defined `std::make_shared<>` instead
 * of `std::make_shared<>` to create objects, to avoid memory alignment
 * problems caused by classes containing Eigen vectors or matrices. Example:
 * \code
 * Foo::Ptr o = std::make_shared<Foo>();
 * \endcode
 * Or using the shorter auxiliary static method `::Create()` for conciseness or
 * to keep compatibility with MRPT 1.5.* code bases:
 * \code
 * Foo::Ptr o = Foo::Create();
 * \endcode
 * If a special memory allocator is needed, use `Foo::CreateAlloc(alloc,...);`.
 * \sa  mrpt::rtti::CObject
 * \ingroup mrpt_rtti_grp
 */
class CObject
{
   protected:
	static mrpt::rtti::TRuntimeClassId* _GetBaseClass();
	static const mrpt::rtti::TRuntimeClassId runtimeClassId;

   public:
	using Ptr = std::shared_ptr<CObject>;
	using ConstPtr = std::shared_ptr<const CObject>;
	using UniquePtr = std::unique_ptr<CObject>;
	using ConstUniquePtr = std::unique_ptr<const CObject>;
	static const mrpt::rtti::TRuntimeClassId& GetRuntimeClassIdStatic();
	/** Returns information about the class of an object in runtime. */
	virtual const mrpt::rtti::TRuntimeClassId* GetRuntimeClass() const
	{
		return CLASS_ID(CObject);
	}

	/** Makes a deep copy of the object and returns a smart pointer to it */
	inline mrpt::rtti::CObject::Ptr duplicateGetSmartPtr() const;

	/** Returns a deep copy (clone) of the object, indepently of its class. */
	virtual CObject* clone() const = 0;

	virtual ~CObject() = default;
};  // End of class def.

inline mrpt::rtti::CObject::Ptr CObject::duplicateGetSmartPtr() const
{
	return mrpt::rtti::CObject::Ptr(this->clone());
}

/** This declaration must be inserted in all CObject classes definition, within
 * the class declaration. */
#define DEFINE_MRPT_OBJECT(class_name)                                    \
	/*! @name RTTI stuff  */                                              \
	/*! @{  */                                                            \
   protected:                                                             \
	static const mrpt::rtti::TRuntimeClassId* _GetBaseClass();            \
	static const mrpt::rtti::TRuntimeClassId runtimeClassId;              \
                                                                          \
   public:                                                                \
	/*! A type for the associated smart pointer */                        \
	using Ptr = std::shared_ptr<class_name>;                              \
	using ConstPtr = std::shared_ptr<const class_name>;                   \
	using UniquePtr = std::unique_ptr<class_name>;                        \
	using ConstUniquePtr = std::unique_ptr<const class_name>;             \
	static constexpr const char* className = #class_name;                 \
	static constexpr auto getClassName()                                  \
	{                                                                     \
		return mrpt::typemeta::literal(#class_name);                      \
	}                                                                     \
	static const mrpt::rtti::TRuntimeClassId& GetRuntimeClassIdStatic();  \
	virtual const mrpt::rtti::TRuntimeClassId* GetRuntimeClass()          \
		const override;                                                   \
	virtual mrpt::rtti::CObject* clone() const override;                  \
	static std::shared_ptr<CObject> CreateObject();                       \
	template <typename... Args>                                           \
	static Ptr Create(Args&&... args)                                     \
	{                                                                     \
		return std::make_shared<class_name>(std::forward<Args>(args)...); \
	}                                                                     \
	template <typename Alloc, typename... Args>                           \
	static Ptr CreateAlloc(const Alloc& alloc, Args&&... args)            \
	{                                                                     \
		return std::allocate_shared<class_name>(                          \
			alloc, std::forward<Args>(args)...);                          \
	}                                                                     \
	template <typename... Args>                                           \
	static UniquePtr CreateUnique(Args&&... args)                         \
	{                                                                     \
		return std::make_unique<class_name>(std::forward<Args>(args)...); \
	}                                                                     \
	/*! @} */                                                             \
   public:

#define INTERNAL_IMPLEMENTS_MRPT_OBJECT(                                      \
	class_name, base, NameSpace, class_registry_name)                         \
	mrpt::rtti::CObject::Ptr NameSpace::class_name::CreateObject()            \
	{                                                                         \
		return std::static_pointer_cast<CObject>(                             \
			std::make_shared<NameSpace::class_name>());                       \
	}                                                                         \
	const mrpt::rtti::TRuntimeClassId* NameSpace::class_name::_GetBaseClass() \
	{                                                                         \
		return CLASS_ID(base);                                                \
	}                                                                         \
	const mrpt::rtti::TRuntimeClassId&                                        \
		NameSpace::class_name::GetRuntimeClassIdStatic()                      \
	{                                                                         \
		return NameSpace::class_name::runtimeClassId;                         \
	}                                                                         \
	const mrpt::rtti::TRuntimeClassId NameSpace::class_name::runtimeClassId = \
		{class_registry_name, NameSpace::class_name::CreateObject,            \
		 &class_name::_GetBaseClass};                                         \
	const mrpt::rtti::TRuntimeClassId*                                        \
		NameSpace::class_name::GetRuntimeClass() const                        \
	{                                                                         \
		return CLASS_ID_NAMESPACE(class_name, NameSpace);                     \
	}                                                                         \
	mrpt::rtti::CObject* NameSpace::class_name::clone() const                 \
	{                                                                         \
		return mrpt::rtti::internal::CopyCtor<std::is_copy_constructible<     \
			NameSpace::class_name>::value>::clone(*this);                     \
	}

/** This must be inserted in all CObject classes implementation files.
 * This version registers calss ns1::Foo as "ns1::Foo", where are
 * IMPLEMENTS_MRPT_OBJECT() makes it for some random name.
 */
#define IMPLEMENTS_MRPT_OBJECT_NS_PREFIX(class_name, base, NameSpace) \
	INTERNAL_IMPLEMENTS_MRPT_OBJECT(                                  \
		class_name, base, NameSpace, #NameSpace "::" #class_name)

/** Must be added to all CObject-derived classes implementation file.
 * This version does NOT include the namespace in the name of the class when
 * registering.
 */
#define IMPLEMENTS_MRPT_OBJECT(class_name, base, NameSpace) \
	INTERNAL_IMPLEMENTS_MRPT_OBJECT(class_name, base, NameSpace, #class_name)

/** This declaration must be inserted in virtual CObject classes
 * definition:
 */
#define DEFINE_VIRTUAL_MRPT_OBJECT(class_name)                           \
	/*! @name RTTI stuff  */                                             \
	/*! @{  */                                                           \
   protected:                                                            \
	static const mrpt::rtti::TRuntimeClassId* _GetBaseClass();           \
	static const mrpt::rtti::TRuntimeClassId runtimeClassId;             \
                                                                         \
   public:                                                               \
	using Ptr = std::shared_ptr<class_name>;                             \
	using ConstPtr = std::shared_ptr<const class_name>;                  \
	virtual const mrpt::rtti::TRuntimeClassId* GetRuntimeClass()         \
		const override;                                                  \
	static const mrpt::rtti::TRuntimeClassId& GetRuntimeClassIdStatic(); \
	/*! @}  */

/** This must be inserted as implementation of some required members for
 *  virtual CObject classes:
 */
#define INTERNAL_IMPLEMENTS_VIRTUAL_MRPT_OBJECT(                               \
	class_name, base_name, NS, registered_name)                                \
	const mrpt::rtti::TRuntimeClassId* NS::class_name::_GetBaseClass()         \
	{                                                                          \
		return CLASS_ID(base_name);                                            \
	}                                                                          \
	const mrpt::rtti::TRuntimeClassId NS::class_name::runtimeClassId = {       \
		registered_name, nullptr, &NS::class_name::_GetBaseClass};             \
	const mrpt::rtti::TRuntimeClassId* NS::class_name::GetRuntimeClass() const \
	{                                                                          \
		return CLASS_ID(class_name);                                           \
	}                                                                          \
	const mrpt::rtti::TRuntimeClassId&                                         \
		NS::class_name::GetRuntimeClassIdStatic()                              \
	{                                                                          \
		return NS::class_name::runtimeClassId;                                 \
	}

#define IMPLEMENTS_VIRTUAL_MRPT_OBJECT_NS_PREFIX(class_name, base, NS) \
	INTERNAL_IMPLEMENTS_VIRTUAL_MRPT_OBJECT(                           \
		class_name, base, NS, #NS "::" #class_name)

#define IMPLEMENTS_VIRTUAL_MRPT_OBJECT(class_name, base, NS) \
	INTERNAL_IMPLEMENTS_VIRTUAL_MRPT_OBJECT(class_name, base, NS, #class_name)

/** Register all pending classes - to be called just before
 * de-serializing an object, for example. After calling this method,
 * pending_class_registers_modified is set to false until
 * pending_class_registers() is invoked.
 */
void registerAllPendingClasses();

/** Creates an object given by its registered name.
 * \sa findRegisteredClass(), registerClass() */
mrpt::rtti::CObject::Ptr classFactory(const std::string& className);

/** @}  */  // end of RTTI

}  // namespace rtti

/** Converts a polymorphic smart pointer Base::Ptr to Derived::Ptr, in a
 * way compatible with MRPT >=1.5.4 and MRPT 2.x series.
 * \ingroup mrpt_rtti_grp
 */
template <typename CAST_TO>
struct ptr_cast
{
	template <typename CAST_FROM_PTR>
	static typename CAST_TO::Ptr from(const CAST_FROM_PTR& ptr)
	{
		return std::dynamic_pointer_cast<CAST_TO>(ptr);
	}
};

}  // namespace mrpt
