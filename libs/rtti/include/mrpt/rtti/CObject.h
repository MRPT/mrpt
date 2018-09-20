/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/safe_pointers.h>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/typemeta/static_string.h>  // literal()
#include <vector>
#include <memory>

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
	mrpt::rtti::CObject* (*ptrCreateObject)();
	/** Gets the base class runtime id. */
	const TRuntimeClassId* (*getBaseClass)();

	// Operations
	mrpt::rtti::CObject* createObject() const;
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
constexpr const mrpt::rtti::TRuntimeClassId* CLASS_ID_impl()
{
	return &T::GetRuntimeClassIdStatic();
}

/** Access to runtime class ID for a defined class name.
 */
#define CLASS_ID(T) mrpt::rtti::CLASS_ID_impl<T>()
// Convert these
#define CLASS_ID_TEMPLATE(class_name, T) mrpt::rtti::CLASS_ID_impl<T>()
#define CLASS_ID_NAMESPACE(class_name, namespaceName) \
	mrpt::rtti::CLASS_ID_impl<namespaceName::class_name>()

template <typename T>
struct IS_CLASS_impl
{
	template <typename PTR>
	static bool check(const PTR& p)
	{
		return p->GetRuntimeClass() == CLASS_ID_impl<T>();
	}
};

/** Evaluates to true if the given pointer to an object (derived from
 * mrpt::rtti::CObject) is of the given class. */
#define IS_CLASS(ptrObj, class_name) \
	mrpt::rtti::IS_CLASS_impl<class_name>::check(ptrObj)

/** Evaluates to true if a pointer to an object (derived from
 * mrpt::rtti::CObject) is an instance of the given class OR any of its
 * derived classes. */
#define IS_DERIVED(ptrObj, class_name) \
	((ptrObj)->GetRuntimeClass()->derivedFrom(CLASS_ID(class_name)))

/** Auxiliary structure used for CObject-based RTTI. \ingroup mrpt_rtti_grp */
struct CLASSINIT
{
	CLASSINIT(const mrpt::rtti::TRuntimeClassId* pNewClass)
	{
		registerClass(pNewClass);
	}
};

/** The virtual base class of all MRPT classes with a unified RTTI system.
 *   For each class named <code>CMyClass</code>, a new type
 * <code>CMyClass::Ptr</code> is defined as a smart pointer (`std::shared_ptr`)
 * suitable for keeping reference-counting smart pointers to objects of the
 * class.
 *
 * It is recommended to use MRPT-defined `mrpt::make_aligned_shared<>` instead
 * of `std::make_shared<>` to create objects, to avoid memory alignment
 * problems caused by classes containing Eigen vectors or matrices. Example:
 * \code
 * CFoo::Ptr o = mrpt::make_aligned_shared<CFoo>();
 * \endcode
 * Or using the shorter auxiliary static method `::Create()` to keep
 * compatibility
 * with MRPT 1.5.* code bases:
 * \code
 * CFoo::Ptr o = CFoo::Create();
 * \endcode
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
	static const mrpt::rtti::TRuntimeClassId& GetRuntimeClassIdStatic();
	/** Returns information about the class of an object in runtime. */
	virtual const mrpt::rtti::TRuntimeClassId* GetRuntimeClass() const
	{
		return CLASS_ID(CObject);
	}

	/** Returns a copy of the object, indepently of its class, as a smart
	 * pointer (the newly created object will exist as long as any copy of this
	 * smart pointer). */
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
	static mrpt::rtti::CLASSINIT _init_##class_name;                      \
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
	static mrpt::rtti::CObject* CreateObject();                           \
	virtual mrpt::rtti::CObject* clone() const override;                  \
	template <typename... Args>                                           \
	static Ptr Create(Args&&... args)                                     \
	{                                                                     \
		return mrpt::make_aligned_shared<class_name>(                     \
			std::forward<Args>(args)...);                                 \
	}                                                                     \
	template <typename... Args>                                           \
	static UniquePtr CreateUnique(Args&&... args)                         \
	{                                                                     \
		return std::make_unique<class_name>(std::forward<Args>(args)...); \
	}                                                                     \
	/*! @} */                                                             \
   public:                                                                \
	MRPT_MAKE_ALIGNED_OPERATOR_NEW

/** This must be inserted in all CObject classes implementation files
 */
#define IMPLEMENTS_MRPT_OBJECT(class_name, base, NameSpace)                   \
	mrpt::rtti::CObject* NameSpace::class_name::CreateObject()                \
	{                                                                         \
		return static_cast<mrpt::rtti::CObject*>(new NameSpace::class_name);  \
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
		{#class_name, NameSpace::class_name::CreateObject,                    \
		 &class_name::_GetBaseClass};                                         \
	const mrpt::rtti::TRuntimeClassId*                                        \
		NameSpace::class_name::GetRuntimeClass() const                        \
	{                                                                         \
		return CLASS_ID_NAMESPACE(class_name, NameSpace);                     \
	}                                                                         \
	mrpt::rtti::CLASSINIT NameSpace::class_name::_init_##class_name(          \
		CLASS_ID(base));                                                      \
	mrpt::rtti::CObject* NameSpace::class_name::clone() const                 \
	{                                                                         \
		return static_cast<mrpt::rtti::CObject*>(                             \
			new NameSpace::class_name(*this));                                \
	}

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
#define IMPLEMENTS_VIRTUAL_MRPT_OBJECT(class_name, base_class_name, NameSpace) \
	const mrpt::rtti::TRuntimeClassId* NameSpace::class_name::_GetBaseClass()  \
	{                                                                          \
		return CLASS_ID(base_class_name);                                      \
	}                                                                          \
	const mrpt::rtti::TRuntimeClassId NameSpace::class_name::runtimeClassId =  \
		{#class_name, nullptr, &NameSpace::class_name::_GetBaseClass};         \
	const mrpt::rtti::TRuntimeClassId*                                         \
		NameSpace::class_name::GetRuntimeClass() const                         \
	{                                                                          \
		return CLASS_ID(class_name);                                           \
	}                                                                          \
	const mrpt::rtti::TRuntimeClassId&                                         \
		NameSpace::class_name::GetRuntimeClassIdStatic()                       \
	{                                                                          \
		return NameSpace::class_name::runtimeClassId;                          \
	}

/** Register all pending classes - to be called just before de-serializing an
 * object, for example.
 * After calling this method, pending_class_registers_modified is set to false
 * until pending_class_registers() is invoked.
 */
void registerAllPendingClasses();

/** Creates an object given by its registered name.
 * \sa findRegisteredClass(), registerClass(), classFactoryPtr() */
mrpt::rtti::CObject* classFactory(const std::string& className);

/** Like classFactory() but returns a smart pointer */
mrpt::rtti::CObject::Ptr classFactoryPtr(const std::string& className);

/** @}  */  // end of RTTI

}  // namespace rtti

/** Converts a polymorphic smart pointer Base::Ptr to Derived::Ptr, in a way
 * compatible with MRPT >=1.5.4 and MRPT 2.x series.
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
