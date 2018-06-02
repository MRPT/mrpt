/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/serialization/CSchemeArchiveBase.h> //no definite use in mind currently
#include <cstdint>

// Make this frwd decl independent of MRPT_HAS_MATLAB in config.h:
/** Forward declaration for mxArray (avoid #including as much as possible to
 * speed up compiling) */
using mxArray = struct mxArray_tag;

namespace mrpt::serialization
{

// /** Used in mrpt::serialization::CSerializable */
// std::string notImplementedErrorString(const std::string& class_name)
// {
// 	return class_name + 
// 			" : class does not support schema based serialization";
// }

/** The virtual base class which provides a unified interface for all persistent
 *objects in MRPT.
 *  Many important properties of this class are inherited from
 *mrpt::rtti::CObject.
 * Refer to the library tutorial: \ref mrpt_serialization_grp
 * \sa CArchive
 * \ingroup mrpt_serialization_grp
 */
class CSerializable : public mrpt::rtti::CObject
{
	friend class CArchive;

	// This must be added to any CObject derived class:
	DEFINE_VIRTUAL_MRPT_OBJECT(CSerializable)

	virtual ~CSerializable() {}
   protected:
	/** @name CSerializable virtual methods
	 *  @{ */
	/** Must return the current versioning number of the object. Start in zero
	 * for new classes, and increments each time there is a change in the stored
	 * format.
	 */
	virtual uint8_t serializeGetVersion() const = 0;
	/** Pure virtual method for writing (serializing) to an abstract archive.
	 *  Users don't call this method directly. Instead, use `stream << object;`.
	 *	\exception std::exception On any I/O error
	 */
	virtual void serializeTo(CArchive& out) const = 0;
	/** Pure virtual method for reading (deserializing) from an abstract
	 *archive. Users don't call this method directly. Instead, use `stream >>
	 *object;`. \param in The input binary stream where the object data must
	 *read from. \param version The version of the object stored in the stream:
	 *use this version number in your code to know how to read the incoming
	 *data. \exception std::exception On any I/O error
	 */
	virtual void serializeFrom(CArchive& in, uint8_t serial_version) = 0;
	/** Virtual method for writing (serializing) to an abstract
	 *  schema based archive. 
	 */
	virtual void serializeTo(CSchemeArchiveBase& out) const
	{
		const std::string err = std::string(this->GetRuntimeClass()->className) +
			std::string(" : class does not support schema based serialization");
		THROW_EXCEPTION(err);
	}
	/** Virtual method for reading (deserializing) from an abstract
	 *  schema based archive.
	 */
	virtual void serializeFrom(CSchemeArchiveBase& in) const
	{
		const std::string err = std::string(this->GetRuntimeClass()->className) +
			std::string(" : class does not support schema based serialization");
		THROW_EXCEPTION(err);
	}
	/** @} */

   public:
	/** Introduces a pure virtual method responsible for writing to a `mxArray`
	 * Matlab object, typically a MATLAB `struct` whose contents are documented
	 * in each derived class. \return A new `mxArray` (caller is responsible of
	 * memory freeing) or nullptr is class does not support conversion to
	 * MATLAB.
	 */
	virtual mxArray* writeToMatlab() const { return nullptr; }
};  // End of class def.

/** \addtogroup noncstream_serialization Non-CStream serialization functions (in
 * #include <mrpt/serializatin/CSerializable.h>)
 * \ingroup mrpt_serialization_grp
 * @{ */

/** Converts (serializes) an MRPT object into an array of bytes.
 * \param o The object to be serialized.
 * \param out_vector The vector which at return will contain the data. Size will
 * be set automatically.
 * \sa OctetVectorToObject, ObjectToString
 */
void ObjectToOctetVector(
	const CSerializable* o, std::vector<uint8_t>& out_vector);

/** Converts back (de-serializes) a sequence of binary data into a MRPT object,
 * without prior information about the object's class.
 * \param in_data The serialized input data representing the object.
 * \param obj The newly created object will be stored in this smart pointer.
 * \exception None On any internal exception, this function returns a nullptr
 * pointer.
 * \sa ObjectToOctetVector, StringToObject
 */
void OctetVectorToObject(
	const std::vector<uint8_t>& in_data, CSerializable::Ptr& obj);

/** @} */
/** This declaration must be inserted in all CSerializable classes definition,
 * within the class declaration. */
#define DEFINE_SCHEMA_SERIALIZABLE()                                              \
   protected:                                                                     \
	/*! @name CSerializable virtual methods for schema based archives*/           \
	/*! @{ */                                                                	  \
	void serializeTo(mrpt::serialization::CSchemeArchiveBase& out) const;\
	void serializeFrom(                                                           \
		mrpt::serialization::CSchemeArchiveBase& in);                    \
/*! @} */																		  \

/** This declaration must be inserted in all CSerializable classes definition,
 * within the class declaration. */
#define DEFINE_SERIALIZABLE(class_name)                                      \
	DEFINE_MRPT_OBJECT(class_name)                                           \
   protected:                                                                \
	/*! @name CSerializable virtual methods */                               \
	/*! @{ */                                                                \
	uint8_t serializeGetVersion() const override;                            \
	void serializeTo(mrpt::serialization::CArchive& out) const override;     \
	void serializeFrom(                                                      \
		mrpt::serialization::CArchive& in, uint8_t serial_version) override; \
/*! @} */

/** This must be inserted in all CSerializable classes implementation files */
#define IMPLEMENTS_SERIALIZABLE(class_name, base, NameSpace) \
	IMPLEMENTS_MRPT_OBJECT(class_name, base, NameSpace)

/** This declaration must be inserted in virtual CSerializable classes
 * definition: */
#define DEFINE_VIRTUAL_SERIALIZABLE(class_name) \
	DEFINE_VIRTUAL_MRPT_OBJECT(class_name)

/** This must be inserted as implementation of some required members for
 *  virtual CSerializable classes:
 */
#define IMPLEMENTS_VIRTUAL_SERIALIZABLE(    \
	class_name, base_class_name, NameSpace) \
	IMPLEMENTS_VIRTUAL_MRPT_OBJECT(class_name, base_class_name, NameSpace)

/** This must be inserted if a custom conversion method for MEX API is
 * implemented in the class */
#define DECLARE_MEX_CONVERSION                           \
	/*! @name Virtual methods for MRPT-MEX conversion */ \
	/*! @{ */                                            \
   public:                                               \
	mxArray* writeToMatlab() const override;             \
/*! @} */

/** This must be inserted if a custom conversion method for MEX API is
 * implemented in the class */
#define DECLARE_MEXPLUS_FROM(complete_type)    \
	namespace mexplus                          \
	{                                          \
	template <typename T>                      \
	mxArray* from(const T& value);             \
	template <>                                \
	mxArray* from(const complete_type& value); \
	}

#define IMPLEMENTS_MEXPLUS_FROM(complete_type) \
	namespace mexplus                          \
	{                                          \
	template <>                                \
	mxArray* from(const complete_type& var)    \
	{                                          \
		return var.writeToMatlab();            \
	}                                          \
	}
}  // namespace mrpt


