/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  MRPT_COBJECT_H
#define  MRPT_COBJECT_H

#include <mrpt/system/memory.h>
#include <mrpt/utils/safe_pointers.h>
#include <vector>

// STL+ library:
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

namespace mrpt
{
	namespace utils
	{
		/** @name RTTI classes and functions
		    @{ */

		class BASE_IMPEXP CObject;

		/** A smart pointer to a CObject object
		  * \note Declared as a class instead of a typedef to avoid multiple defined symbols when linking dynamic libs.
		  * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CObjectPtr : public stlplus::smart_ptr_clone<CObject>
		{
			typedef stlplus::smart_ptr_clone<CObject> BASE;
		public:
			inline CObjectPtr() : BASE() {}
			explicit inline CObjectPtr(const CObject& data) :  BASE(data) {}
			explicit inline CObjectPtr(CObject* data) :  BASE(data) { }
			inline CObjectPtr& operator=(const CObject& data) { BASE::operator=(data); return *this; }
			inline CObjectPtr& operator=(const CObjectPtr& r) { BASE::operator=(r); return *this; }
		};

		/** A structure that holds runtime class type information. Use CLASS_ID(<class_name>) to get a reference to the class_name's TRuntimeClassId descriptor.
		  * \ingroup mrpt_base_grp
		  */
		struct BASE_IMPEXP TRuntimeClassId
		{
			const char* className;
			/** Create an object of the related class, or NULL if it is virtual. */
			mrpt::utils::CObject*			(*ptrCreateObject)();
			/** Gets the base class runtime id. */
			const TRuntimeClassId*		(*getBaseClass)();

			// Operations
			mrpt::utils::CObject*		createObject() const;
			bool	derivedFrom(const TRuntimeClassId* pBaseClass) const;
			bool	derivedFrom(const char* pBaseClass_name) const;

		};

		/** A wrapper class for a "TRuntimeClassId *", well-defined with respect to copy operators and constructors. \ingroup mrpt_base_grp
		  */
		typedef safe_ptr<TRuntimeClassId> TRuntimeClassIdPtr;

		/** Register a class into the MRPT internal list of "CSerializable" descendents.
		  *  Used internally in the macros DEFINE_SERIALIZABLE, etc...
		  * \sa getAllRegisteredClasses
		  */
		void BASE_IMPEXP registerClass(const mrpt::utils::TRuntimeClassId* pNewClass);

		/** Mostly for internal use within mrpt sources, to handle exceptional cases with multiple serialization names for backward compatibility (CMultiMetricMaps, CImage,...)
		  */
		void  BASE_IMPEXP registerClassCustomName(const char*customName, const TRuntimeClassId* pNewClass);

		/** Returns a list with all the classes registered in the system through mrpt::utils::registerClass.
		  * \sa registerClass, findRegisteredClass
		  */
		std::vector<const mrpt::utils::TRuntimeClassId*> BASE_IMPEXP getAllRegisteredClasses();

		/** Return info about a given class by its name, or NULL if the class is not registered
		  * \sa registerClass, getAllRegisteredClasses
		  */
		const TRuntimeClassId BASE_IMPEXP * findRegisteredClass(const std::string &className);


		/** Access to runtime class ID for a defined class name.
		  */
		#define CLASS_ID(class_name) static_cast<const mrpt::utils::TRuntimeClassId*>(&class_name::class##class_name)

		/** Access to runtime class ID for a defined class name.
		  */
		#define CLASS_ID_NAMESPACE(class_name,namespaceName) static_cast<const mrpt::utils::TRuntimeClassId*>(&namespaceName::class_name::class##class_name)

		/** Access to runtime class ID for a defined template class name.
		  */
		#define CLASS_ID_TEMPLATE(class_name,T) static_cast<const mrpt::utils::TRuntimeClassId*>(& template <class T> class_name<T>::class##class_name)

		/** Evaluates to true if the given pointer to an object (derived from mrpt::utils::CSerializable) is of the given class. */
		#define IS_CLASS( ptrObj, class_name )  ((ptrObj)->GetRuntimeClass()==CLASS_ID(class_name))

		/** Evaluates to true if the given pointer to an object (derived from mrpt::utils::CSerializable) is an instance of the given class or any of its derived classes. */
		#define IS_DERIVED( ptrObj, class_name )  ((ptrObj)->GetRuntimeClass()->derivedFrom(CLASS_ID(class_name)))

		/** Auxiliary structure used for CObject-based RTTI. \ingroup mrpt_base_grp */
		struct BASE_IMPEXP CLASSINIT
		{
			CLASSINIT(const mrpt::utils::TRuntimeClassId* pNewClass)
			{
				registerClass(pNewClass);
			}
		};


		/** The virtual base class of all MRPT classes with a unified RTTI system.
		 *   For each class named <code>CMyClass</code>, a new type named <code>CMyClassPtr</code> will be created as a smart pointer suitable for
		 *    keeping referencing count smart pointers to objects of that class. By default the base class of all these smart pointers is CObjectPtr.
		 * \sa  mrpt::utils::CSerializable \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CObject
		{
		protected:
			static  mrpt::utils::TRuntimeClassId* _GetBaseClass();
		public:
			static const mrpt::utils::TRuntimeClassId classCObject;

			/** Returns information about the class of an object in runtime. */
			virtual const mrpt::utils::TRuntimeClassId* GetRuntimeClass() const
			{
				return CLASS_ID(CObject);
			}

			/** Returns a copy of the object, indepently of its class. */
			virtual CObject *duplicate() const = 0;

			/** Returns a copy of the object, indepently of its class, as a smart pointer (the newly created object will exist as long as any copy of this smart pointer). */
			inline mrpt::utils::CObjectPtr duplicateGetSmartPtr() const { return mrpt::utils::CObjectPtr( this->duplicate() ); }

			/** Cloning interface for smart pointers */
			inline CObject *clone() const { return duplicate(); }

			virtual ~CObject() {  }

		}; // End of class def.


		/** Just like DEFINE_MRPT_OBJECT but with DLL export/import linkage keywords. Note: The replication of macro arguments is to avoid errors with empty macro arguments */
		#define DEFINE_MRPT_OBJECT_CUSTOM_LINKAGE(class_name, _STATIC_LINKAGE_, _VIRTUAL_LINKAGE_) \
			/*! @name RTTI stuff  */ \
			/*! @{  */ \
		protected: \
			_STATIC_LINKAGE_ const mrpt::utils::TRuntimeClassId* _GetBaseClass(); \
			_STATIC_LINKAGE_ mrpt::utils::CLASSINIT _init_##class_name;\
		public: \
			/*! A typedef for the associated smart pointer */ \
			typedef class_name##Ptr SmartPtr; \
			_STATIC_LINKAGE_ mrpt::utils::TRuntimeClassId  class##class_name; \
			_STATIC_LINKAGE_ const mrpt::utils::TRuntimeClassId *classinfo; \
			_VIRTUAL_LINKAGE_ const mrpt::utils::TRuntimeClassId* GetRuntimeClass() const MRPT_OVERRIDE; \
			_STATIC_LINKAGE_ mrpt::utils::CObject* CreateObject(); \
			_STATIC_LINKAGE_ class_name##Ptr Create(); \
			_VIRTUAL_LINKAGE_ mrpt::utils::CObject *duplicate() const MRPT_OVERRIDE; \
			/*! @} */ \
		public: \
			MRPT_MAKE_ALIGNED_OPERATOR_NEW \

		/** This declaration must be inserted in all CObject classes definition, within the class declaration. */
		#define DEFINE_MRPT_OBJECT(class_name) \
			DEFINE_MRPT_OBJECT_CUSTOM_LINKAGE(class_name, static /*none*/, virtual /*none*/)

		// This macro is a workaround to avoid possibly empty arguments to MACROS (when _LINKAGE_ evals to nothing...)
		#define DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE(class_name, base_name, _LINKAGE_ )  DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, base_name, _LINKAGE_ class_name)
		#define DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE(class_name, base_name, _LINKAGE_ ) DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(class_name, base_name, _LINKAGE_ class_name)

		// Use this one when there is NO import/export macro:
		#define DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_NO_LINKAGE(class_name, base_name) DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, base_name, class_name)
		#define DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_NO_LINKAGE(class_name, base_name) DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(class_name, base_name, class_name)

		/**  This declaration must be inserted in all CObject classes definition, before the class declaration.  */
		#define DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, base_name, class_name_LINKAGE_ ) \
			class  class_name_LINKAGE_; \
			struct class_name_LINKAGE_##Ptr;

		/**  This declaration must be inserted in all CObject classes definition, after the class declaration.  */
		#define DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(class_name, base_name, class_name_LINKAGE_ ) \
			/*! The smart pointer type for the associated class */ \
			struct class_name_LINKAGE_##Ptr : public base_name##Ptr \
			{ \
				typedef class_name value_type; \
				inline class_name##Ptr() : base_name##Ptr(static_cast<base_name*>(NULL)) { } \
				inline explicit class_name##Ptr(class_name* p) : base_name##Ptr( static_cast<base_name*>(p) ) { } \
				inline explicit class_name##Ptr(const base_name##Ptr & p) : base_name##Ptr(p) { if(!p.null()) ASSERTMSG_( p->GetRuntimeClass()->derivedFrom(#class_name),::mrpt::format("Wrong typecasting of smart pointers: %s -> %s",p->GetRuntimeClass()->className, #class_name) )  } \
				inline explicit class_name##Ptr(const mrpt::utils::CObjectPtr & p) : base_name##Ptr(p) { if(!p.null())ASSERTMSG_( p->GetRuntimeClass()->derivedFrom(#class_name),::mrpt::format("Wrong typecasting of smart pointers: %s -> %s",p->GetRuntimeClass()->className, #class_name) )  } \
				inline void setFromPointerDoNotFreeAtDtor(const class_name* p) { this->set(const_cast<mrpt::utils::CObject*>(static_cast<const mrpt::utils::CObject*>(p))); m_holder->increment(); } \
				/*! Return the internal plain C++ pointer */ \
				inline class_name * pointer() { return dynamic_cast<class_name*>(base_name##Ptr::pointer()); } \
				/*! Return the internal plain C++ pointer (const) */ \
				inline const class_name * pointer() const { return dynamic_cast<const class_name*>(base_name##Ptr::pointer()); } \
				inline class_name* operator ->(void) { return dynamic_cast<class_name*>( base_name##Ptr::operator ->() ); } \
				inline const class_name* operator ->(void) const { return dynamic_cast<const class_name*>( base_name##Ptr::operator ->() ); } \
				inline class_name& operator *(void) { return *dynamic_cast<class_name*>( base_name##Ptr::operator ->() ); } \
				inline const class_name& operator *(void) const { return *dynamic_cast<const class_name*>( base_name##Ptr::operator ->() ); } \
			};


		// This macro is a workaround to avoid possibly empty arguments to MACROS (when _LINKAGE_ evals to nothing...)
		#define DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE(class_name,_LINKAGE_)  DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE2(class_name, _LINKAGE_ class_name)
		#define DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE(class_name,_LINKAGE_) DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE2(class_name, _LINKAGE_ class_name)

		// Use this macro when there is NO export/import macro:
		#define DEFINE_MRPT_OBJECT_PRE_NO_LINKAGE(class_name) DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE2(class_name, class_name)
		#define DEFINE_MRPT_OBJECT_POST_NO_LINKAGE(class_name) DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE2(class_name, class_name)

		// This one is almost identical to the one above, but without a member:
		/**  This declaration must be inserted in all CObject classes definition, before the class declaration. */
		#define DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE2(class_name,class_name_LINKAGE_) \
			struct class_name_LINKAGE_##Ptr;

		/**  This declaration must be inserted in all CObject classes definition, after the class declaration. */
		#define DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE2(class_name,class_name_LINKAGE_) \
			/*! The smart pointer type for the associated class */ \
			struct class_name_LINKAGE_##Ptr : public mrpt::utils::CObjectPtr \
			{ \
				inline class_name##Ptr() : mrpt::utils::CObjectPtr(static_cast<mrpt::utils::CObject*>(NULL)) { } \
				inline explicit class_name##Ptr(class_name* p) : mrpt::utils::CObjectPtr( static_cast<mrpt::utils::CObject*>(p) ) { } \
				inline explicit class_name##Ptr(const mrpt::utils::CObjectPtr & p) : mrpt::utils::CObjectPtr(p) { ASSERTMSG_( p->GetRuntimeClass()->derivedFrom(#class_name),::mrpt::format("Wrong typecasting of smart pointers: %s -> %s",p->GetRuntimeClass()->className, #class_name) )  } \
				inline void setFromPointerDoNotFreeAtDtor(const class_name* p) { this->set(const_cast<mrpt::utils::CObject*>(static_cast<const mrpt::utils::CObject*>(p))); m_holder->increment(); } \
				/*! Return the internal plain C++ pointer */ \
				inline class_name * pointer() { return dynamic_cast<class_name*>(mrpt::utils::CObjectPtr::pointer()); } \
				/*! Return the internal plain C++ pointer (const) */ \
				inline const class_name * pointer() const { return dynamic_cast<const class_name*>(mrpt::utils::CObjectPtr::pointer()); } \
				inline class_name* operator ->(void) { return dynamic_cast<class_name*>( mrpt::utils::CObjectPtr::operator ->() ); } \
				inline const class_name* operator ->(void) const { return dynamic_cast<const class_name*>( mrpt::utils::CObjectPtr::operator ->() ); } \
				inline class_name& operator *(void) { return *dynamic_cast<class_name*>( mrpt::utils::CObjectPtr::operator ->() ); } \
				inline const class_name& operator *(void) const { return *dynamic_cast<const class_name*>( mrpt::utils::CObjectPtr::operator ->() ); } \
			};

		/**  This declaration must be inserted in all CObject classes definition, before the class declaration.
		  */
		#define DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE(class_name, base_name) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE(class_name, base_name, BASE_IMPEXP )

		/**  This declaration must be inserted in all CObject classes definition, before the class declaration.
		  */
		#define DEFINE_MRPT_OBJECT_PRE(class_name)  DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE(class_name, BASE_IMPEXP )  // This macro is valid for classes within mrpt-base only.
		#define DEFINE_MRPT_OBJECT_POST(class_name) DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE(class_name, BASE_IMPEXP )  // This macro is valid for classes within mrpt-base only.

		/** This must be inserted in all CObject classes implementation files
		  */
		#define IMPLEMENTS_MRPT_OBJECT(class_name, base,NameSpace) \
			mrpt::utils::CObject* NameSpace::class_name::CreateObject() \
				{ return static_cast<mrpt::utils::CObject*>( new NameSpace::class_name ); } \
			NameSpace::class_name##Ptr NameSpace::class_name::Create() \
				{ return NameSpace::class_name##Ptr( new NameSpace::class_name ); } \
			const mrpt::utils::TRuntimeClassId* NameSpace::class_name::_GetBaseClass() \
				{ return CLASS_ID(base); } \
			mrpt::utils::TRuntimeClassId NameSpace::class_name::class##class_name = { \
				#class_name, NameSpace::class_name::CreateObject, &class_name::_GetBaseClass }; \
			const mrpt::utils::TRuntimeClassId *NameSpace::class_name::classinfo = & NameSpace::class_name::class##class_name; \
			const mrpt::utils::TRuntimeClassId* NameSpace::class_name::GetRuntimeClass() const \
			{ return CLASS_ID_NAMESPACE(class_name,NameSpace); } \
			mrpt::utils::CLASSINIT NameSpace::class_name::_init_##class_name(CLASS_ID(base)); \
			mrpt::utils::CObject * NameSpace::class_name::duplicate() const \
			{ return static_cast<mrpt::utils::CObject*>( new NameSpace::class_name(*this) ); }


		/** This declaration must be inserted in virtual CSerializable classes definition:
		  */
		#define DEFINE_VIRTUAL_MRPT_OBJECT(class_name) \
		/*! @name RTTI stuff  */ \
		/*! @{  */ \
		protected: \
			static const mrpt::utils::TRuntimeClassId* _GetBaseClass(); \
		public: \
			static const mrpt::utils::TRuntimeClassId class##class_name; \
			virtual const mrpt::utils::TRuntimeClassId* GetRuntimeClass() const MRPT_OVERRIDE; \
			friend class mrpt::utils::CStream; \
		/*! @}  */ \

		/** This must be inserted as implementation of some required members for
		  *  virtual CSerializable classes:
		  */
		#define IMPLEMENTS_VIRTUAL_MRPT_OBJECT(class_name, base_class_name,NameSpace) \
			const mrpt::utils::TRuntimeClassId* class_name::_GetBaseClass() \
				{ return CLASS_ID(base_class_name); } \
			const mrpt::utils::TRuntimeClassId class_name::class##class_name = { \
				#class_name, NULL, &class_name::_GetBaseClass }; \
			const mrpt::utils::TRuntimeClassId* class_name::GetRuntimeClass() const \
				{ return CLASS_ID(class_name); }

		/** @}  */   // end of RTTI

		/** Register all pending classes - to be called just before de-serializing an object, for example.
		  * After calling this method, pending_class_registers_modified is set to false until pending_class_registers() is invoked.
 		  */
		void BASE_IMPEXP registerAllPendingClasses();

	} // End of namespace
} // End of namespace

// JL: I want these operators to reside in std so STL algorithms can always find them.
namespace std
{
	/**  This operator enables comparing two smart pointers with "==" to test whether they point to the same object.
	  */
	template <typename T,typename C, typename COUNTER>
	inline bool operator == ( const stlplus::smart_ptr_base<T,C,COUNTER>&a,const stlplus::smart_ptr_base<T,C,COUNTER>&b) {
		return a.aliases(b);
	}
	/**  This operator enables comparing two smart pointers with "!=" to test whether they don't point to the same object.
	  */
	template <typename T,typename C, typename COUNTER>
	inline bool operator != ( const stlplus::smart_ptr_base<T,C,COUNTER>&a,const stlplus::smart_ptr_base<T,C,COUNTER>&b) {
		return !a.aliases(b);
	}
}

#endif
