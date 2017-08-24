/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef MRPT_COBJECT_H
#define MRPT_COBJECT_H

#include <mrpt/system/memory.h>
#include <mrpt/utils/safe_pointers.h>
#include <mrpt/utils/aligned_allocator.h>
#include <vector>
#include <memory>

namespace mrpt
{
namespace utils
{
/** @name RTTI classes and functions
	@{ */
class BASE_IMPEXP CObject;

/** A structure that holds runtime class type information. Use
 * CLASS_ID(<class_name>) to get a reference to the class_name's TRuntimeClassId
 * descriptor.
  * \ingroup mrpt_base_grp
  */
struct BASE_IMPEXP TRuntimeClassId
{
	using Ptr = safe_ptr<TRuntimeClassId>;
	const char* className;
	/** Create an object of the related class, or nullptr if it is virtual. */
	mrpt::utils::CObject* (*ptrCreateObject)();
	/** Gets the base class runtime id. */
	const TRuntimeClassId* (*getBaseClass)();

	// Operations
	mrpt::utils::CObject* createObject() const;
	bool derivedFrom(const TRuntimeClassId* pBaseClass) const;
	bool derivedFrom(const char* pBaseClass_name) const;
};

/** Register a class into the MRPT internal list of "CSerializable" descendents.
  *  Used internally in the macros DEFINE_SERIALIZABLE, etc...
  * \sa getAllRegisteredClasses
  */
void BASE_IMPEXP registerClass(const mrpt::utils::TRuntimeClassId* pNewClass);

/** Mostly for internal use within mrpt sources, to handle exceptional cases
 * with multiple serialization names for backward compatibility
 * (CMultiMetricMaps, CImage,...)
  */
void BASE_IMPEXP registerClassCustomName(
	const char* customName, const TRuntimeClassId* pNewClass);

/** Returns a list with all the classes registered in the system through
 * mrpt::utils::registerClass.
  * \sa registerClass, findRegisteredClass
  */
std::vector<const mrpt::utils::TRuntimeClassId*> BASE_IMPEXP
	getAllRegisteredClasses();

/** Like getAllRegisteredClasses(), but filters the list to only include
 * children clases of a given base one.
  * \sa getAllRegisteredClasses(), getAllRegisteredClassesChildrenOf()  */
std::vector<const TRuntimeClassId*> BASE_IMPEXP
	getAllRegisteredClassesChildrenOf(const TRuntimeClassId* parent_id);

/** Return info about a given class by its name, or nullptr if the class is not
 * registered
  * \sa registerClass, getAllRegisteredClasses
  */
const TRuntimeClassId BASE_IMPEXP* findRegisteredClass(
	const std::string& className);

template <typename T>
constexpr const mrpt::utils::TRuntimeClassId* CLASS_ID_impl()
{
	return &T::runtimeClassId;
}

/** Access to runtime class ID for a defined class name.
 */
#define CLASS_ID(T) mrpt::utils::CLASS_ID_impl<T>()
// Convert these
#define CLASS_ID_TEMPLATE(class_name, T) mrpt::utils::CLASS_ID_impl<T>()
#define CLASS_ID_NAMESPACE(class_name, namespaceName) \
	mrpt::utils::CLASS_ID_impl<namespaceName::class_name>()

template <typename T>
struct IS_CLASS_impl
{
	template <typename PTR>
	static bool check(const PTR p)
	{
		return p->GetRuntimeClass() == CLASS_ID_impl<T>();
	}
};

/** Evaluates to true if the given pointer to an object (derived from
 * mrpt::utils::CSerializable) is of the given class. */
#define IS_CLASS(ptrObj, class_name) \
	mrpt::utils::IS_CLASS_impl<class_name>::check(ptrObj)

/** Evaluates to true if the given pointer to an object (derived from
 * mrpt::utils::CSerializable) is an instance of the given class or any of its
 * derived classes. */
#define IS_DERIVED(ptrObj, class_name) \
	((ptrObj)->GetRuntimeClass()->derivedFrom(CLASS_ID(class_name)))

/** Auxiliary structure used for CObject-based RTTI. \ingroup mrpt_base_grp */
struct BASE_IMPEXP CLASSINIT
{
	CLASSINIT(const mrpt::utils::TRuntimeClassId* pNewClass)
	{
		registerClass(pNewClass);
	}
};

/** This is useful for checking ::Ptr types.
  * I'm surprised it's not defined in <memory>
  */
template <class T>
struct is_shared_ptr : std::false_type
{
};
template <class T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type
{
};

/** The virtual base class of all MRPT classes with a unified RTTI system.
 *   For each class named <code>CMyClass</code>, a new type named
 * <code>CMyClass::Ptr</code> will be created as a smart pointer suitable for
 *    keeping referencing count smart pointers to objects of that class. By
 * default the base class of all these smart pointers is CObject::Ptr.
 *
 * It is recommended to use MRPT-defined `mrpt::make_aligned_shared<>` instead
 * of `std::make_shared<>` to create objects, to avoid memory alignment
 * problems caused by classes containing Eigen vectors or matrices. Example:
 * \code
 * CFoo::Ptr o = mrpt::make_aligned_shared<CFoo>();
 * \endcode
 * \sa  mrpt::utils::CSerializable \ingroup mrpt_base_grp
 */
class BASE_IMPEXP CObject
{
   protected:
	static mrpt::utils::TRuntimeClassId* _GetBaseClass();

   public:
	using Ptr = std::shared_ptr<CObject>;
	using ConstPtr = std::shared_ptr<const CObject>;
	static const mrpt::utils::TRuntimeClassId runtimeClassId;

	/** Returns information about the class of an object in runtime. */
	virtual const mrpt::utils::TRuntimeClassId* GetRuntimeClass() const
	{
		return CLASS_ID(CObject);
	}

	/** Returns a copy of the object, indepently of its class, as a smart
	 * pointer (the newly created object will exist as long as any copy of this
	 * smart pointer). */
	inline mrpt::utils::CObject::Ptr duplicateGetSmartPtr() const;

	/** Returns a deep copy (clone) of the object, indepently of its class. */
	virtual CObject* clone() const = 0;

	virtual ~CObject() {}
};  // End of class def.

/** A smart pointer to a CObject object
  * \note Declared as a class instead of a typedef to avoid multiple defined
 * symbols when linking dynamic libs.
  * \ingroup mrpt_base_grp
  */
inline mrpt::utils::CObject::Ptr CObject::duplicateGetSmartPtr() const
{
	return mrpt::utils::CObject::Ptr(this->clone());
}

/** Just like DEFINE_MRPT_OBJECT but with DLL export/import linkage keywords.
 * Note: The replication of macro arguments is to avoid errors with empty macro
 * arguments */
#define DEFINE_MRPT_OBJECT_CUSTOM_LINKAGE(                                  \
	class_name, _STATIC_LINKAGE_, _VIRTUAL_LINKAGE_)                        \
	/*! @name RTTI stuff  */                                                \
	/*! @{  */                                                              \
   protected:                                                               \
	_STATIC_LINKAGE_ const mrpt::utils::TRuntimeClassId* _GetBaseClass();   \
	_STATIC_LINKAGE_ mrpt::utils::CLASSINIT _init_##class_name;             \
                                                                            \
   public:                                                                  \
	/*! A typedef for the associated smart pointer */                       \
	using Ptr = std::shared_ptr<class_name>;                                \
	using ConstPtr = std::shared_ptr<const class_name>;                     \
	_STATIC_LINKAGE_ mrpt::utils::TRuntimeClassId runtimeClassId;           \
	_STATIC_LINKAGE_ const mrpt::utils::TRuntimeClassId* classinfo;         \
	_VIRTUAL_LINKAGE_ const mrpt::utils::TRuntimeClassId* GetRuntimeClass() \
		const override;                                                     \
	_STATIC_LINKAGE_ mrpt::utils::CObject* CreateObject();                  \
	_VIRTUAL_LINKAGE_ mrpt::utils::CObject* clone() const override;         \
	template <typename... Args>                                             \
	static Ptr Create(Args&&... args)                                       \
	{                                                                       \
		return mrpt::make_aligned_shared<class_name>(                       \
			std::forward<Args>(args)...);                                   \
	}                                                                       \
	/*! @} */                                                               \
   public:                                                                  \
	MRPT_MAKE_ALIGNED_OPERATOR_NEW

/** This declaration must be inserted in all CObject classes definition, within
 * the class declaration. */
#define DEFINE_MRPT_OBJECT(class_name) \
	DEFINE_MRPT_OBJECT_CUSTOM_LINKAGE( \
		class_name, static /*none*/, virtual /*none*/)

// This macro is a workaround to avoid possibly empty arguments to MACROS (when
// _LINKAGE_ evals to nothing...)
#define DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE( \
	class_name, base_name, _LINKAGE_)                \
	DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(    \
		class_name, base_name, _LINKAGE_ class_name)

// Use this one when there is NO import/export macro:
#define DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_NO_LINKAGE(class_name, base_name) \
	DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(                             \
		class_name, base_name, class_name)

/**  This declaration must be inserted in all CObject classes definition, before
 * the class declaration.  */

/**  This declaration must be inserted in all CObject classes definition, after
 * the class declaration.  */
#define DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2( \
	class_name, base_name, class_name_LINKAGE_)

// This macro is a workaround to avoid possibly empty arguments to MACROS (when
// _LINKAGE_ evals to nothing...)
#define DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE(class_name, _LINKAGE_) \
	DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE2(class_name, _LINKAGE_ class_name)

// Use this macro when there is NO export/import macro:
#define DEFINE_MRPT_OBJECT_POST_NO_LINKAGE(class_name) \
	DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE2(class_name, class_name)

// This one is almost identical to the one above, but without a member:
/**  This declaration must be inserted in all CObject classes definition, before
 * the class declaration. */

/**  This declaration must be inserted in all CObject classes definition, after
 * the class declaration. */
/** MRPT 1.X backwards-compatible smart pointer name (to be removed in a future
 * version of MRPT! Use CFoo::Ptr instead) */
#ifdef MRPT_1X_BACKCOMPATIB_SMARTPTR_NAMES
#define DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE2( \
	class_name, class_name_LINKAGE_)             \
	using class_name##Ptr = class_name::Ptr;
#else
#define DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE2(class_name, class_name_LINKAGE_)
#endif

/**  This declaration must be inserted in all CObject classes definition, before
 * the class declaration.
  */

/**  This declaration must be inserted in all CObject classes definition, before
 * the class declaration.
  */
#define DEFINE_MRPT_OBJECT_POST(class_name) \
	DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE( \
		class_name, BASE_IMPEXP)  // This macro is valid for classes within
// mrpt-base only.

/** This must be inserted in all CObject classes implementation files
  */
#define IMPLEMENTS_MRPT_OBJECT(class_name, base, NameSpace)                    \
	mrpt::utils::CObject* NameSpace::class_name::CreateObject()                \
	{                                                                          \
		return static_cast<mrpt::utils::CObject*>(new NameSpace::class_name);  \
	}                                                                          \
	const mrpt::utils::TRuntimeClassId* NameSpace::class_name::_GetBaseClass() \
	{                                                                          \
		return CLASS_ID(base);                                                 \
	}                                                                          \
	mrpt::utils::TRuntimeClassId NameSpace::class_name::runtimeClassId = {     \
		#class_name, NameSpace::class_name::CreateObject,                      \
		&class_name::_GetBaseClass};                                           \
	const mrpt::utils::TRuntimeClassId* NameSpace::class_name::classinfo =     \
		&NameSpace::class_name::runtimeClassId;                                \
	const mrpt::utils::TRuntimeClassId*                                        \
		NameSpace::class_name::GetRuntimeClass() const                         \
	{                                                                          \
		return CLASS_ID_NAMESPACE(class_name, NameSpace);                      \
	}                                                                          \
	mrpt::utils::CLASSINIT NameSpace::class_name::_init_##class_name(          \
		CLASS_ID(base));                                                       \
	mrpt::utils::CObject* NameSpace::class_name::clone() const                 \
	{                                                                          \
		return static_cast<mrpt::utils::CObject*>(                             \
			new NameSpace::class_name(*this));                                 \
	}

/** This declaration must be inserted in virtual CSerializable classes
 * definition:
  */
#define DEFINE_VIRTUAL_MRPT_OBJECT(class_name)                    \
	/*! @name RTTI stuff  */                                      \
	/*! @{  */                                                    \
   protected:                                                     \
	static const mrpt::utils::TRuntimeClassId* _GetBaseClass();   \
                                                                  \
   public:                                                        \
	using Ptr = std::shared_ptr<class_name>;                      \
	using ConstPtr = std::shared_ptr<const class_name>;           \
	static const mrpt::utils::TRuntimeClassId runtimeClassId;     \
	virtual const mrpt::utils::TRuntimeClassId* GetRuntimeClass() \
		const override;                                           \
	friend class mrpt::utils::CStream;                            \
/*! @}  */

/** This must be inserted as implementation of some required members for
  *  virtual CSerializable classes:
  */
#define IMPLEMENTS_VIRTUAL_MRPT_OBJECT(class_name, base_class_name, NameSpace) \
	const mrpt::utils::TRuntimeClassId* class_name::_GetBaseClass()            \
	{                                                                          \
		return CLASS_ID(base_class_name);                                      \
	}                                                                          \
	const mrpt::utils::TRuntimeClassId class_name::runtimeClassId = {          \
		#class_name, nullptr, &class_name::_GetBaseClass};                     \
	const mrpt::utils::TRuntimeClassId* class_name::GetRuntimeClass() const    \
	{                                                                          \
		return CLASS_ID(class_name);                                           \
	}

/** @}  */  // end of RTTI

/** Register all pending classes - to be called just before de-serializing an
 * object, for example.
  * After calling this method, pending_class_registers_modified is set to false
 * until pending_class_registers() is invoked.
  */
void BASE_IMPEXP registerAllPendingClasses();

}  // End of namespace

/** Converts a smart pointer Base::Ptr to Derived::Ptr, in a way compatible
* with MRPT >=1.5.4 and MRPT 2.x series.
* \ingroup mrpt_base_grp
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

}  // End of namespace

#endif
