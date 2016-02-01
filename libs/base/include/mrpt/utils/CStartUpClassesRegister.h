/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CStartUpClassesRegister_H
#define  CStartUpClassesRegister_H

#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
	namespace utils
	{
		/** A helper class that automatically register at start up a custom function to register all the CObject-derived classes in a given MRPT library or user application.
		  *   Usage:
		  *  \code
		  *    void registerAllMyClasses()
		  *    {
		  *      registerClass(CLASS_ID( CMyClass1 ) );
		  *      ...
		  *    }
		  *
		  *    CStartUpClassesRegister   doReg( &registerAllMyClasses );
		  *
		  *  \endcode
		 * \ingroup mrpt_base_grp
		  */
		class BASE_IMPEXP CStartUpClassesRegister
		{
		public:
			/** Read the global description of mrpt::utils::CStartUpClassesRegister */
			CStartUpClassesRegister(void (*ptr_register_func)() );
			~CStartUpClassesRegister();

			int do_nothing(); //!<< dummy method to allow introducing dependences and avoid the compiler removing the class in static linking

		private:
			void (*m_ptr_register_func)();  //!< An internal copy of the functor.

			int m_dummy_var;
		};


	} // End of namespace
} // End of namespace

#endif
