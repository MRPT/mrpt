/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CUNCOPIABLE_H
#define  CUNCOPIABLE_H

#include <mrpt/base/link_pragmas.h>

namespace mrpt
{
	namespace utils
	{
		/** The base class of classes that cannot be copied: compile-time errors will be issued on any copy operation.
		 *   An example:
		 *
		 *  \code
		 *   class MyFancyClass : public mrpt::utils::CUncopiable
		 *   {
		 *    public:
		 *     ...
		 *   };
		 *  \endcode
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CUncopiable
		{
		private:
			CUncopiable(const CUncopiable &);  // This doesn't need to be implemented anywhere
			const CUncopiable& operator =(const CUncopiable &);   // This doesn't need to be implemented anywhere
		public:
			CUncopiable() {  }
		}; // End of class def.

	} // End of namespace
} // end of namespace
#endif
