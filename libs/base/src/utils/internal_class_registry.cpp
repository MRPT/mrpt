/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
				mrpt::synch::CCriticalSectionLocker lock(&m_cs);
				registeredClasses[className] = &id;
			}

			const TRuntimeClassId *Get(const std::string &className)
			{
				mrpt::synch::CCriticalSectionLocker lock(&m_cs);
				return registeredClasses[className];
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
			CClassRegistry()
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

