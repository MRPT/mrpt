/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
