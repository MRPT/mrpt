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
#ifndef  CStartUpClassesRegister_H
#define  CStartUpClassesRegister_H

#include <mrpt/utils/utils_defs.h>

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
