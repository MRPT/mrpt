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
#ifndef  CSERIALIZABLE_H
#define  CSERIALIZABLE_H

#include <mrpt/utils/CObject.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/safe_pointers.h>

#include <set>
#include <list>

namespace mrpt
{
	/** Classes for serialization, sockets, ini-file manipulation, streams, list of properties-values, timewatch, extensions to STL.
	  * \ingroup mrpt_base_grp
	  */
	namespace utils
	{
		DEFINE_MRPT_OBJECT_PRE( CSerializable )

		/** The virtual base class which provides a unified interface for all persistent objects in MRPT.
		 *  Many important properties of this class are inherited from mrpt::utils::CObject. See that class for more details.
		 *	 Refer to the tutorial about <a href="http://www.mrpt.org/Serialization" >serialization</a> in the wiki.
		 * \sa CStream
		 * \ingroup mrpt_base_grp
		 */
		class BASE_IMPEXP CSerializable : public mrpt::utils::CObject
		{
			// This must be added to any CObject derived class:
			DEFINE_VIRTUAL_MRPT_OBJECT( CSerializable )

			virtual ~CSerializable() { }

		protected:
			 /** Introduces a pure virtual method responsible for writing to a CStream.
			  *  This can not be used directly be users, instead use "stream << object;"
			  *   for writing it to a stream.
			  * \param out The output binary stream where object must be dumped.
			  * \param getVersion If NULL, the object must be dumped. If not, only the
			  *		version of the object dump must be returned in this pointer. This enables
			  *     the versioning of objects dumping and backward compatibility with previously
			  *     stored data.
			  *	\exception std::exception On any error, see CStream::WriteBuffer
			  * \sa CStream
			  */
			virtual void  writeToStream(mrpt::utils::CStream &out, int *getVersion) const = 0;

			 /** Introduces a pure virtual method responsible for loading from a CStream
			  *  This can not be used directly be users, instead use "stream >> object;"
			  *   for reading it from a stream or "stream >> object_ptr;" if the class is
			  *   unknown apriori.
			  * \param in The input binary stream where the object data must read from.
			  * \param version The version of the object stored in the stream: use this version
			  *                number in your code to know how to read the incoming data.
			  *	\exception std::exception On any error, see CStream::ReadBuffer
			  * \sa CStream
			  */
			virtual void  readFromStream(mrpt::utils::CStream &in, int version) = 0;
		}; // End of class def.


		/** @name Non-streaming serialization functions
		@{ */

		/** Used to pass MRPT objects into a CORBA-like object (strings). See doc about "Integration with BABEL".
		 * \param o The object to be serialized.
		 * \return The string containing the binay version of object.
		 * \sa StringToObject, <a href="http://www.mrpt.org/Integration_with_BABEL" >Integration with BABEL</a>
		 */
		std::string BASE_IMPEXP ObjectToString(const CSerializable *o);

		/** Used to pass CORBA-like objects (strings) into a MRPT object.
		 * \param str An string generated with ObjectToString
		 * \param obj A currently empty pointer, where a pointer to the newly created object will be stored.
		 * \exception None On any internal exception, this function returns NULL.
		 * \sa ObjectToString, <a href="http://www.mrpt.org/Integration_with_BABEL" >Integration with BABEL</a>
		 */
		void BASE_IMPEXP StringToObject(const std::string &str, CSerializablePtr &obj);

		/** Converts (serializes) an MRPT object into an array of bytes.
		 * \param o The object to be serialized.
		 * \param out_vector The vector which at return will contain the data. Size will be set automatically.
		 * \sa OctetVectorToObject, ObjectToString
		 */
		void BASE_IMPEXP ObjectToOctetVector(const CSerializable *o, vector_byte & out_vector);

		/** Converts back (de-serializes) a sequence of binary data into a MRPT object, without prior information about the object's class.
		 * \param in_data The serialized input data representing the object.
		 * \param obj The newly created object will be stored in this smart pointer.
		 * \exception None On any internal exception, this function returns a NULL pointer.
		 * \sa ObjectToOctetVector, StringToObject
		 */
		void BASE_IMPEXP OctetVectorToObject(const vector_byte & in_data, CSerializablePtr &obj);

		/** Converts (serializes) an MRPT object into an array of bytes within a std::string, without codifying to avoid NULL characters.
		 *  This is therefore more efficient than ObjectToString
		 * \param o The object to be serialized.
		 * \param out_vector The string which at return will contain the data. Size will be set automatically.
		 * \sa RawStringToObject, ObjectToOctetVector
		 */
		void BASE_IMPEXP ObjectToRawString(const CSerializable *o, std::string & out_str);

		/** Converts back (de-serializes) a sequence of binary data within a std::string into a MRPT object, without prior information about the object's class.
		 * \param in_data The serialized input data representing the object.
		 * \param obj The newly created object will be stored in this smart pointer.
		 * \exception None On any internal exception, this function returns a NULL pointer.
		 * \sa ObjectToRawString
		 */
		void BASE_IMPEXP RawStringToObject(const std::string & in_str, CSerializablePtr &obj);

		/** @} */



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

		// These ones are not needed since typedef's of "std::vector<>" are automatically supported
		//MRPT_DECLARE_TTYPENAME_MAP_FOR_STDVECTOR(vector_signed_byte )
		//MRPT_DECLARE_TTYPENAME_MAP_FOR_STDVECTOR(vector_signed_word)
		//MRPT_DECLARE_TTYPENAME_MAP_FOR_STDVECTOR(vector_int)
		//MRPT_DECLARE_TTYPENAME_MAP_FOR_STDVECTOR(vector_long)
		//MRPT_DECLARE_TTYPENAME_MAP_FOR_STDVECTOR(vector_byte)
		//MRPT_DECLARE_TTYPENAME_MAP_FOR_STDVECTOR(vector_word)
		//MRPT_DECLARE_TTYPENAME_MAP_FOR_STDVECTOR(vector_uint)
        //#if MRPT_WORD_SIZE!=32  // If it's 32 bit, size_t <=> uint32_t
		//MRPT_DECLARE_TTYPENAME_MAP_FOR_STDVECTOR(vector_size_t)
        //#endif


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


		/** This declaration must be inserted in all CSerializable classes definition, within the class declaration.
		  */
		#define DEFINE_SERIALIZABLE(class_name) \
			DEFINE_MRPT_OBJECT(class_name) \
		protected: \
			/*! @name CSerializable virtual methods */ \
			/*! @{ */ \
			void  writeToStream(mrpt::utils::CStream &out, int *getVersion) const;\
			void  readFromStream(mrpt::utils::CStream &in, int version); \
			/*! @} */

		/**  This declaration must be inserted in all CSerializable classes definition, before the class declaration.
		  */
		#define DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE(class_name,_LINKAGE_) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, CSerializable, _LINKAGE_ class_name) \
			_LINKAGE_ ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj);


		/**  This declaration must be inserted in all CSerializable classes definition, before the class declaration.
		  */
		#define DEFINE_SERIALIZABLE_PRE(class_name) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, CSerializable, BASE_IMPEXP class_name) \
			BASE_IMPEXP ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj);

		/**  This declaration must be inserted in all CSerializable classes definition, before the class declaration.
		  */
		#define DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(class_name, base_name, _LINKAGE_ ) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, base_name, _LINKAGE_ class_name) \
			_LINKAGE_ ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj);


		/**  This declaration must be inserted in all CSerializable classes definition, before the class declaration.
		  */
		#define DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE(class_name, base_name) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE(class_name, base_name, BASE_IMPEXP ) \
			BASE_IMPEXP ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj);

		/** This must be inserted in all CSerializable classes implementation files:
		  */
		#define IMPLEMENTS_SERIALIZABLE(class_name, base,NameSpace) \
			IMPLEMENTS_MRPT_OBJECT(class_name, base,NameSpace) \
			mrpt::utils::CStream& NameSpace::operator>>(mrpt::utils::CStream& in, NameSpace::class_name##Ptr &pObj) \
			{ pObj = NameSpace::class_name##Ptr( in.ReadObject() ); return in; }

		/** This declaration must be inserted in virtual CSerializable classes definition:
		  */
		#define DEFINE_VIRTUAL_SERIALIZABLE(class_name) \
			DEFINE_VIRTUAL_MRPT_OBJECT(class_name)

		/** This must be inserted as implementation of some required members for
		  *  virtual CSerializable classes:
		  */
		#define IMPLEMENTS_VIRTUAL_SERIALIZABLE(class_name, base_class_name,NameSpace) \
			IMPLEMENTS_VIRTUAL_MRPT_OBJECT(class_name, base_class_name,NameSpace) \
			mrpt::utils::CStream& NameSpace::operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj) \
			{ pObj = class_name##Ptr( in.ReadObject() ); return in; }


	} // End of namespace
} // End of namespace

#endif
