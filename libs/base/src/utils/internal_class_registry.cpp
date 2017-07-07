/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CObject.h>

#include <map>
#include <cstdarg>
#include <mutex>
#include <atomic>
#include <iostream>

#include "internal_class_registry.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;


/*---------------------------------------------------------------
					STATIC GLOBAL VARIABLES
 ---------------------------------------------------------------*/
volatile bool mrpt::utils::pending_class_registers_modified = false;

// Creation on first call pattern:
std::atomic<int> & mrpt::utils::pending_class_registers_count()
{
	static std::atomic<int> cnt(0);
	return cnt;
}

// Creation on first call pattern:
CThreadSafeQueue<TRegisterFunction>	 &mrpt::utils::pending_class_registers()
{
	pending_class_registers_modified = true;
	static CThreadSafeQueue<TRegisterFunction> lst;
	return lst;
}


namespace mrpt
{
	namespace utils
	{
		typedef std::map<std::string,const TRuntimeClassId*> TClassnameToRuntimeId;

		/** A singleton with the central registry for CSerializable run-time classes: users do not use this class in any direct way.
	      * \note Class is thread-safe.
		  */
		class BASE_IMPEXP CClassRegistry
		{
		public:
			/**  The unique access point point to the singleton instance.
			  */
			static CClassRegistry& Instance()
			{
				static CClassRegistry obj;
				return obj;
			}

			void Add( const std::string &className, const TRuntimeClassId &id )
			{
				m_being_modified=true;
				{
					std::unique_lock<std::mutex> lk(m_cs);

					// Sanity check: don't allow registering twice the same class name!
					const auto it = registeredClasses.find(className);
					if (it != registeredClasses.cend()) {
						if (it->second != &id) {
							std::cerr << mrpt::format("[MRPT class registry] Warning: overwriting already registered className=`%s` with different `TRuntimeClassId`!\n", className.c_str());
						}
					}
					registeredClasses[className] = &id;
				}
				m_being_modified=false;
			}

			const TRuntimeClassId *Get(const std::string &className)
			{
				// Optimization to avoid the costly lock() in virtually all situations:
				bool has_to_unlock = false;
				if (m_being_modified)
				{
					m_cs.lock();
					has_to_unlock = true;
				}
				const TRuntimeClassId *ret = registeredClasses[className];
				if (has_to_unlock) m_cs.unlock();
				return ret;
			}

			std::vector<const TRuntimeClassId*> getListOfAllRegisteredClasses()
			{
				std::unique_lock<std::mutex> lk(m_cs);

				std::vector<const TRuntimeClassId*> ret;
				for (TClassnameToRuntimeId::iterator it=registeredClasses.begin();it!=registeredClasses.end();++it)
					ret.push_back( it->second );
				return ret;
			}

		private:
			// PRIVATE constructor
			CClassRegistry() : m_being_modified(false)
			{
			}
			// PRIVATE destructor
			~CClassRegistry() { }

			// This must be static since we can be called from C startup
			// functions and it cannot be assured that classesKeeper will be
			// initialized before other classes that call it...
			TClassnameToRuntimeId			registeredClasses;
			std::mutex        m_cs;
			std::atomic<bool> m_being_modified;

		};

	}	// End of namespace
}	// End of namespace


/** Register all pending classes - to be called just before de-serializing an object, for example.
*/
void mrpt::utils::registerAllPendingClasses()
{
	if (!pending_class_registers_modified) return; // Quick return

	while( pending_class_registers_count()!=0 )
	{
		TRegisterFunction *ptrToPtr = pending_class_registers().get();
		--pending_class_registers_count();

		// Call it:
		if((*ptrToPtr)!=nullptr)
		{
			(*(*ptrToPtr)) ();
			delete ptrToPtr;
		}
	}
	pending_class_registers_modified = false;
}



/*---------------------------------------------------------------
					RegisterClass
 ---------------------------------------------------------------*/
void utils::registerClass(const TRuntimeClassId* pNewClass)
{
	// Register it:
	CClassRegistry::Instance().Add( std::string( pNewClass->className ), *pNewClass);

	// Automatically register all classes when the first one is registered.
	registerAllPendingClasses();
}

/** For internal use within mrpt sources, and only in exceptional cases (CMultiMetricMaps, CImage,...)
  */
void utils::registerClassCustomName(const char*customName, const TRuntimeClassId* pNewClass)
{
	// Register it:
	CClassRegistry::Instance().Add( customName, *pNewClass);

	// Automatically register all classes when the first one is registered.
	registerAllPendingClasses();
}

/*---------------------------------------------------------------
					getAllRegisteredClasses
 ---------------------------------------------------------------*/
std::vector<const TRuntimeClassId*> utils::getAllRegisteredClasses()
{
	return CClassRegistry::Instance().getListOfAllRegisteredClasses();
}

std::vector<const TRuntimeClassId*> utils::getAllRegisteredClassesChildrenOf(const TRuntimeClassId* parent_id)
{
	std::vector<const TRuntimeClassId*> res;
	const auto lst = mrpt::utils::getAllRegisteredClasses();
	for (const auto &c : lst) {
		if (c->derivedFrom(parent_id) && c!=parent_id) {
			res.push_back(c);
		}
	}
	return res;
}

/*---------------------------------------------------------------
					findRegisteredClass
 ---------------------------------------------------------------*/
const TRuntimeClassId *mrpt::utils::findRegisteredClass(const std::string &className)
{
	return CClassRegistry::Instance().Get( className );
}
