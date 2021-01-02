/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "rtti-precomp.h"  // Precompiled headers

#include <mrpt/rtti/CObject.h>

#include <atomic>
#include <cstdarg>
#include <iostream>
#include <map>
#include <mutex>

#include "internal_class_registry.h"

using namespace mrpt::rtti;

/*---------------------------------------------------------------
					STATIC GLOBAL VARIABLES
 ---------------------------------------------------------------*/
bool mrpt::rtti::pending_class_registers_modified = false;

// Creation on first call pattern:
std::atomic<int>& mrpt::rtti::pending_class_registers_count()
{
	static std::atomic<int> cnt(0);
	return cnt;
}

// Creation on first call pattern:
queue_register_functions_t& mrpt::rtti::pending_class_registers()
{
	pending_class_registers_modified = true;
	static queue_register_functions_t lst;
	return lst;
}

namespace mrpt::rtti
{
using TClassnameToRuntimeId = std::map<std::string, const TRuntimeClassId*>;

/** A singleton with the central registry for CSerializable run-time classes:
 * users do not use this class in any direct way.
 * \note Class is thread-safe.
 */
class CClassRegistry
{
   public:
	/**  The unique access point point to the singleton instance.
	 */
	static CClassRegistry& Instance()
	{
		static CClassRegistry obj;
		return obj;
	}

	void Add(const std::string& className, const TRuntimeClassId& id)
	{
		m_being_modified = true;
		{
			std::unique_lock<std::mutex> lk(m_cs);

			// Sanity check: don't allow registering twice the same class name!
			const auto it = m_ns_classes.find(className);
			if (it != m_ns_classes.cend())
			{
				if (it->second != &id)
				{
					std::cerr << mrpt::format(
						"[MRPT class registry] Warning: overwriting already "
						"registered className=`%s` with different "
						"`TRuntimeClassId`!\n",
						className.c_str());
				}
			}
			m_ns_classes[className] = &id;

			// Also register without NS (backwards compatible datasets):
			m_no_ns_classes[stripNamespace(className)] = &id;
		}
		m_being_modified = false;
	}

	const TRuntimeClassId* Get(
		const std::string& className, const bool allow_ignore_namespace)
	{
		// Optimization to avoid the costly lock() in virtually all situations:
		bool has_to_unlock = false;
		if (m_being_modified)
		{
			m_cs.lock();
			has_to_unlock = true;
		}
		const TRuntimeClassId* ret = nullptr;
		const auto itEntry = m_ns_classes.find(className);
		if (itEntry != m_ns_classes.end())
		{
			// found:
			ret = itEntry->second;
		}
		else if (allow_ignore_namespace)
		{
			// 2nd attempt: search for class name only:
			const auto itEntry2 =
				m_no_ns_classes.find(stripNamespace(className));
			if (itEntry2 != m_no_ns_classes.end())
			{
				// found:
				ret = itEntry2->second;
			}
		}
		if (has_to_unlock) m_cs.unlock();
		return ret;
	}

	std::vector<const TRuntimeClassId*> getListOfAllRegisteredClasses()
	{
		std::unique_lock<std::mutex> lk(m_cs);

		std::vector<const TRuntimeClassId*> ret;
		for (auto& registeredClasse : m_ns_classes)
			ret.push_back(registeredClasse.second);
		return ret;
	}

   private:
	static std::string stripNamespace(const std::string& n)
	{
		std::string ret = n;
		const auto pos = ret.rfind("::");
		if (pos != std::string::npos)
		{
			return ret.substr(pos + 2);
		}
		return ret;
	}

	// This must be static since we can be called from C startup
	// functions and it cannot be assured that classesKeeper will be
	// initialized before other classes that call it...
	TClassnameToRuntimeId m_ns_classes;

	// The auxiliary copy of "m_ns_classes", w/o namespace prefixes:
	TClassnameToRuntimeId m_no_ns_classes;

	std::mutex m_cs;
	std::atomic<bool> m_being_modified{false};
};

}  // namespace mrpt::rtti

/** Register all pending classes - to be called just before de-serializing an
 * object, for example.
 */
void mrpt::rtti::registerAllPendingClasses()
{
	if (!pending_class_registers_modified) return;  // Quick return

	while (pending_class_registers_count() != 0)
	{
		TRegisterFunction ptrToPtr = nullptr;
		pending_class_registers().get(ptrToPtr);
		--pending_class_registers_count();

		// Call it:
		if (ptrToPtr != nullptr)
		{
			(*ptrToPtr)();
		}
	}
	pending_class_registers_modified = false;
}

/*---------------------------------------------------------------
					RegisterClass
 ---------------------------------------------------------------*/
void mrpt::rtti::registerClass(const TRuntimeClassId* pNewClass)
{
	// Register it:
	if (pNewClass && pNewClass->className)
	{
		CClassRegistry::Instance().Add(
			std::string(pNewClass->className), *pNewClass);
	}
	else
	{
		fprintf(
			stderr,
			"[mrpt::rtti::registerClass] Warning: Invoked with a nullptr "
			"classname (?).\n");
	}

	// Automatically register all classes when the first one is registered.
	registerAllPendingClasses();
}

/** For internal use within mrpt sources, and only in exceptional cases
 * (CMultiMetricMaps, CImage,...)
 */
void mrpt::rtti::registerClassCustomName(
	const char* customName, const TRuntimeClassId* pNewClass)
{
	// Register it:
	CClassRegistry::Instance().Add(customName, *pNewClass);

	// Automatically register all classes when the first one is registered.
	registerAllPendingClasses();
}

std::vector<const TRuntimeClassId*> mrpt::rtti::getAllRegisteredClasses()
{
	return CClassRegistry::Instance().getListOfAllRegisteredClasses();
}

std::vector<const TRuntimeClassId*>
	mrpt::rtti::getAllRegisteredClassesChildrenOf(
		const TRuntimeClassId* parent_id)
{
	std::vector<const TRuntimeClassId*> res;
	const auto lst = mrpt::rtti::getAllRegisteredClasses();
	for (const auto& c : lst)
	{
		if (c->derivedFrom(parent_id) && c != parent_id)
		{
			res.push_back(c);
		}
	}
	return res;
}

/*---------------------------------------------------------------
					findRegisteredClass
 ---------------------------------------------------------------*/
const TRuntimeClassId* mrpt::rtti::findRegisteredClass(
	const std::string& className, const bool allow_ignore_namespace)
{
	return CClassRegistry::Instance().Get(className, allow_ignore_namespace);
}
