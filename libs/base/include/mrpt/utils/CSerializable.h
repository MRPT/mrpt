/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CSERIALIZABLE_H
#define  CSERIALIZABLE_H

#include <mrpt/utils/CObject.h>
#include <mrpt/utils/TTypeName.h>
#include <mrpt/utils/types_simple.h>

#if MRPT_HAS_MATLAB
typedef struct mxArray_tag mxArray; //!< Forward declaration for mxArray (avoid #including as much as possible to speed up compiling)
#endif

namespace mrpt
{
	namespace utils {
		class CStream;
	}

	/** Classes for serialization, sockets, ini-file manipulation, streams, list of properties-values, timewatch, extensions to STL.
	  * \ingroup mrpt_base_grp
	  */
	namespace utils
	{
		DEFINE_MRPT_OBJECT_PRE( CSerializable )

		/** The virtual base class which provides a unified interface for all persistent objects in MRPT.
		 *  Many important properties of this class are inherited from mrpt::utils::CObject. See that class for more details.
		 *	 Refer to the tutorial about <a href="http://www.mrpt.org/Serialization" >serialization</a> online.
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

		public:

			/** Introduces a pure virtual method responsible for writing to a `mxArray` Matlab object,
			  * typically a MATLAB `struct` whose contents are documented in each derived class.
			  * \return A new `mxArray` (caller is responsible of memory freeing) or NULL is class does not support conversion to MATLAB.
			  */
#if MRPT_HAS_MATLAB
			virtual mxArray* writeToMatlab() const { return NULL; }
#endif
		}; // End of class def.

		DEFINE_MRPT_OBJECT_POST( CSerializable )

		/** \addtogroup noncstream_serialization Non-CStream serialization functions (in #include <mrpt/utils/CSerializable.h>)
		  * \ingroup mrpt_base_grp
		  * @{ */

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

		/** Like DEFINE_SERIALIZABLE, but for template classes that need the DLL imp/exp keyword in Windows. */
		#define DEFINE_SERIALIZABLE_CUSTOM_LINKAGE(class_name, _VOID_LINKAGE_, _STATIC_LINKAGE_, _VIRTUAL_LINKAGE_ ) \
			DEFINE_MRPT_OBJECT_CUSTOM_LINKAGE(class_name, _STATIC_LINKAGE_, _VIRTUAL_LINKAGE_ ) \
		protected: \
			/*! @name CSerializable virtual methods */ \
			/*! @{ */ \
			_VOID_LINKAGE_ writeToStream(mrpt::utils::CStream &out, int *getVersion) const MRPT_OVERRIDE;\
			_VOID_LINKAGE_ readFromStream(mrpt::utils::CStream &in, int version) MRPT_OVERRIDE; \
			/*! @} */

		/** This declaration must be inserted in all CSerializable classes definition, within the class declaration. */
		#define DEFINE_SERIALIZABLE(class_name) \
			DEFINE_SERIALIZABLE_CUSTOM_LINKAGE(class_name, void /*no extra linkage keyword*/, static /*none*/,virtual /*none*/ )

		/**  This declaration must be inserted in all CSerializable classes definition, before the class declaration.
		  */
		#define DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE(class_name,_LINKAGE_) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, mrpt::utils::CSerializable, _LINKAGE_ class_name) \
			_LINKAGE_ ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj);

		#define DEFINE_SERIALIZABLE_POST_CUSTOM_LINKAGE(class_name,_LINKAGE_) \
			DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(class_name, mrpt::utils::CSerializable, _LINKAGE_ class_name)

		/**  This declaration must be inserted in all CSerializable classes definition, before the class declaration.
		  */
		#define DEFINE_SERIALIZABLE_PRE(class_name) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, mrpt::utils::CSerializable, BASE_IMPEXP class_name) \
			BASE_IMPEXP ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj);

		#define DEFINE_SERIALIZABLE_POST(class_name) \
			DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(class_name, mrpt::utils::CSerializable, BASE_IMPEXP class_name)

		/**  This declaration must be inserted in all CSerializable classes definition, before the class declaration.
		  */
		#define DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(class_name, base_name, _LINKAGE_ ) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE2(class_name, base_name, _LINKAGE_ class_name) \
			_LINKAGE_ ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj);

		#define DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(class_name, base_name, _LINKAGE_ ) \
			DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE2(class_name, base_name, _LINKAGE_ class_name)

		/**  This declaration must be inserted in all CSerializable classes definition, before the class declaration. */
		#define DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE(class_name, base_name) \
			DEFINE_MRPT_OBJECT_PRE_CUSTOM_BASE_LINKAGE(class_name, base_name, BASE_IMPEXP ) \
			BASE_IMPEXP ::mrpt::utils::CStream& operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj);

		#define DEFINE_SERIALIZABLE_POST_CUSTOM_BASE(class_name, base_name) \
			DEFINE_MRPT_OBJECT_POST_CUSTOM_BASE_LINKAGE(class_name, base_name, BASE_IMPEXP ) \

		/** This must be inserted in all CSerializable classes implementation files */
		#define IMPLEMENTS_SERIALIZABLE(class_name, base,NameSpace) \
			IMPLEMENTS_MRPT_OBJECT(class_name, base,NameSpace) \
			mrpt::utils::CStream& NameSpace::operator>>(mrpt::utils::CStream& in, NameSpace::class_name##Ptr &pObj) \
			{ pObj = NameSpace::class_name##Ptr( in.ReadObject() ); return in; }

		/** This declaration must be inserted in virtual CSerializable classes definition: */
		#define DEFINE_VIRTUAL_SERIALIZABLE(class_name) \
			DEFINE_VIRTUAL_MRPT_OBJECT(class_name)

		/** This must be inserted as implementation of some required members for
		  *  virtual CSerializable classes:
		  */
		#define IMPLEMENTS_VIRTUAL_SERIALIZABLE(class_name, base_class_name,NameSpace) \
			IMPLEMENTS_VIRTUAL_MRPT_OBJECT(class_name, base_class_name,NameSpace) \
			mrpt::utils::CStream& NameSpace::operator>>(mrpt::utils::CStream& in, class_name##Ptr &pObj) \
			{ pObj = class_name##Ptr( in.ReadObject() ); return in; }

		/** This must be inserted if a custom conversion method for MEX API is implemented in the class */
		#if MRPT_HAS_MATLAB
			#define DECLARE_MEX_CONVERSION \
			/*! @name Virtual methods for MRPT-MEX conversion */ \
			/*! @{ */ \
			public: \
				virtual mxArray* writeToMatlab() const; \
			/*! @} */
		#else
			#define DECLARE_MEX_CONVERSION //Empty
		#endif

		/** This must be inserted if a custom conversion method for MEX API is implemented in the class */
		#if MRPT_HAS_MATLAB
			#define DECLARE_MEXPLUS_FROM( complete_type ) \
			namespace mexplus \
			{ \
				template <typename T> \
				mxArray* from(const T& value); \
				template <> \
				mxArray* from(const complete_type& value); \
			}

			#define IMPLEMENTS_MEXPLUS_FROM( complete_type ) \
			namespace mexplus \
			{ \
				template <> \
				mxArray* from(const complete_type& var) \
				{ \
					return var.writeToMatlab(); \
				} \
			}
		#else
			#define DECLARE_MEXPLUS_FROM(complete_type) //Empty
			#define IMPLEMENTS_MEXPLUS_FROM(complete_type) //Empty
		#endif

	} // End of namespace
} // End of namespace

#endif
