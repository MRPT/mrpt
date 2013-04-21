/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/utils/CObject.h>

#include <map>
#include <iostream>
#include <cstdarg>

#include "internal_class_registry.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::system;
using namespace std;


/*---------------------------------------------------------------
					STATIC GLOBAL VARIABLES
 ---------------------------------------------------------------*/
volatile bool mrpt::utils::pending_class_registers_modified = false;

// Creation on first call pattern:
CAtomicCounter	& mrpt::utils::pending_class_registers_count()
{
	static CAtomicCounter cnt(0);
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
					mrpt::synch::CCriticalSectionLocker lock(&m_cs);
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
					m_cs.enter();
					has_to_unlock = true;
				}
				const TRuntimeClassId *ret = registeredClasses[className];
				if (has_to_unlock) m_cs.leave();
				return ret;
			}

			std::vector<const TRuntimeClassId*> getListOfAllRegisteredClasses()
			{
				mrpt::synch::CCriticalSectionLocker lock(&m_cs);

				std::vector<const TRuntimeClassId*>	ret;
				for (TClassnameToRuntimeId::iterator it=registeredClasses.begin();it!=registeredClasses.end();++it)
					ret.push_back( it->second );
				return ret;
			}

		private:
			// PRIVATE constructor
			CClassRegistry() : m_being_modified(false)
			{
				// A good place to put this... it will be always invoked without the user needing to call it ;-)
				mrpt::system::registerFatalExceptionHandlers();
			}
			// PRIVATE destructor
			~CClassRegistry() { }

			// This must be static since we can be called from C startup
			// functions and it cannot be assured that classesKeeper will be
			// initialized before other classes that call it...
			TClassnameToRuntimeId			registeredClasses;
			mrpt::synch::CCriticalSection 	m_cs;
			volatile bool                   m_being_modified;

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
		if((*ptrToPtr)!=NULL)
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

/*---------------------------------------------------------------
					findRegisteredClass
 ---------------------------------------------------------------*/
const TRuntimeClassId *mrpt::utils::findRegisteredClass(const std::string &className)
{
	return CClassRegistry::Instance().Get( className );
}

