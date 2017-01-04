/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  TTYPENAME_H
#define  TTYPENAME_H

#include <mrpt/utils/mrpt_stdint.h>    // compiler-independent version of "stdint.h"
#include <string>

namespace mrpt
{
	namespace utils
	{
		/** @name Conversion of type to string at compile time
		  * IMPORTANT: See also the implementation of Serialization for STL containers in <mrpt/utils/stl_serialization.h>
		@{ */

		/** A template to obtain the type of its argument as a string at compile time.
		  *  It works with all classes derived from  CSerializable, plus many specializations for the plain data types (bool, double, uint8_t, etc...)
		  *   For example:
		  *  \code
		  *     cout << TTypeName<double>::get() << endl;                          // "double"
		  *   	cout << TTypeName<CPose2D>::get() << endl;                         // "CPose2D"
		  *   	cout << TTypeName<mrpt::maps::COccupancyGridMap2D>::get() << endl; // "COccupancyGridMap2D"
		  *  \endcode
		  *
		  *  Users can extend this for custom structs/classes with the macro DECLARE_CUSTOM_TTYPENAME:
		  *  \code
		  *     class MyClass { ... };
		  *     DECLARE_CUSTOM_TTYPENAME(MyClass)
		  *     cout << TTypeName<MyClass>::get() << endl;                          // "MyClass"
		  *  \endcode
		  *
		  *  The following types are NOT ALLOWED since they have platform-dependant sizes:
		  *  - int, unsigned int
		  *  - long, unsigned long
		  *  - short, unsigned short
		  *  - size_t
		  *
		  */
		template<typename T>
		struct TTypeName
		{
			static std::string get() {
				return std::string( T::classinfo->className );
			}
		};

		/** Identical to MRPT_DECLARE_TTYPENAME but intended for user code.
		  * MUST be placed at the GLOBAL namespace.
		  */
		#define DECLARE_CUSTOM_TTYPENAME(_TYPE) \
			namespace mrpt { namespace utils { MRPT_DECLARE_TTYPENAME(_TYPE) } }

		#define MRPT_DECLARE_TTYPENAME(_TYPE) \
			template<> struct TTypeName <_TYPE > { \
				static std::string get() { return std::string(#_TYPE); } };

		#define MRPT_DECLARE_TTYPENAME_NAMESPACE(_TYPE,__NS) \
			template<> struct TTypeName <__NS :: _TYPE > { \
				static std::string get() { return std::string(#_TYPE); } };

		#define MRPT_DECLARE_TTYPENAME_PTR(_TYPE) \
			template<> struct TTypeName <_TYPE##Ptr> { \
			static std::string get() { return TTypeName<_TYPE>::get(); }	};

		#define MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(_TYPE,__NS) \
			template<> struct TTypeName <__NS :: _TYPE##Ptr> { \
			static std::string get() { return TTypeName<__NS :: _TYPE>::get(); }	};

		MRPT_DECLARE_TTYPENAME(bool)
		MRPT_DECLARE_TTYPENAME(double)
		MRPT_DECLARE_TTYPENAME(float)
		MRPT_DECLARE_TTYPENAME(uint64_t)
		MRPT_DECLARE_TTYPENAME(int64_t)
		MRPT_DECLARE_TTYPENAME(uint32_t)
		MRPT_DECLARE_TTYPENAME(int32_t)
		MRPT_DECLARE_TTYPENAME(uint16_t)
		MRPT_DECLARE_TTYPENAME(int16_t)
		MRPT_DECLARE_TTYPENAME(uint8_t)
		MRPT_DECLARE_TTYPENAME(int8_t)

		/** @} */

	} // End of namespace
} // End of namespace

#endif
