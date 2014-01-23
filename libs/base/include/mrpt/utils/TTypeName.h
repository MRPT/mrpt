/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  TTYPENAME_H
#define  TTYPENAME_H

#include <mrpt/utils/utils_defs.h>

namespace mrpt
{
	namespace utils
	{
		/** @name Conversion of type to string at compile time
		  * IMPORTANT: See also the implementation of Serialization for STL containers in mrpt/utils/stl_extensions.h
		@{ */

		/** A template to obtain the type of its argument as a string at compile time.
		  *  It works with all classes derived from  CSerializable, plus many specializations for the plain data types (bool, double, uint8_t, etc...)
		  *   For example:
		  *  \code
		  *     cout << TTypeName<double>::get() << endl;                          // "double"
		  *   	cout << TTypeName<CPose2D>::get() << endl;                         // "CPose2D"
		  *   	cout << TTypeName<mrpt::slam::COccupancyGridMap2D>::get() << endl; // "COccupancyGridMap2D"
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

		#define MRPT_DECLARE_TTYPENAME_PTR(_TYPE) \
			template<> struct TTypeName <_TYPE##Ptr> { \
			static std::string get() { return TTypeName<_TYPE>::get(); }	};

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

		MRPT_DECLARE_TTYPENAME(std::string)


		#define MRPT_DECLARE_TTYPENAME_CONTAINER(_CONTAINER) \
			template< typename V > \
			struct TTypeName <_CONTAINER<V> > { \
				static std::string get() { \
					return std::string( #_CONTAINER )+std::string("<")+std::string( TTypeName<V>::get() ) + std::string(">"); \
				} \
			};

		MRPT_DECLARE_TTYPENAME_CONTAINER( std::vector )
		MRPT_DECLARE_TTYPENAME_CONTAINER( std::deque )
		MRPT_DECLARE_TTYPENAME_CONTAINER( std::list )
		MRPT_DECLARE_TTYPENAME_CONTAINER( std::set )

		// These ones act as a "translation" between vector_XXX types and their base classes:
		#define MRPT_DECLARE_TTYPENAME_MAP_FOR_VECTOR(_CONT) \
			template<> struct TTypeName <_CONT> : TTypeName<std::vector<_CONT::Scalar> > { };

		MRPT_DECLARE_TTYPENAME_MAP_FOR_VECTOR(vector_float)
		MRPT_DECLARE_TTYPENAME_MAP_FOR_VECTOR(vector_double)


		#define MRPT_DECLARE_TTYPENAME_CONTAINER_ASSOC(_CONTAINER) \
			template< typename K, typename V > \
			struct TTypeName <_CONTAINER<K,V> > { \
				static std::string get() { \
					return std::string( #_CONTAINER )+std::string("<")+std::string( TTypeName<K>::get() )+ std::string(",")+std::string( TTypeName<V>::get() )+std::string(">"); \
				} \
			};

		MRPT_DECLARE_TTYPENAME_CONTAINER_ASSOC( std::map )
		MRPT_DECLARE_TTYPENAME_CONTAINER_ASSOC( std::multimap )


		template< typename T1, typename T2 >
		struct TTypeName <std::pair<T1,T2> >	{
			static std::string get() {
				return std::string("std::pair<")+std::string( TTypeName<T1>::get() )+ std::string(",")+std::string( TTypeName<T2>::get() )+std::string(">");
			}
		};

		/** @} */

	} // End of namespace
} // End of namespace

#endif
