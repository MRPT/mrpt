/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/utils/CStartUpClassesRegister.h>

#include "internal_class_registry.h"


using namespace mrpt;
using namespace mrpt::utils;


/*---------------------------------------------------------------
                Constructor
 ---------------------------------------------------------------*/
CStartUpClassesRegister::CStartUpClassesRegister( void (*ptr_register_func)() ) :
	m_ptr_register_func( ptr_register_func ),
	m_dummy_var(0)
{
	// Put the function into the thread-safe list of pending registrations:
	pending_class_registers().push( new TRegisterFunction(ptr_register_func) );
	++pending_class_registers_count();
}

CStartUpClassesRegister::~CStartUpClassesRegister()
{
	m_ptr_register_func=NULL;
}

int CStartUpClassesRegister::do_nothing()
{
	return ++m_dummy_var;
}
