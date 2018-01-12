/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/img/TColor.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CLoadableOptions.h>

/* STD */
#include <cstdint>

using namespace boost::python;
using namespace mrpt::config;
using namespace mrpt::img;
using namespace mrpt::rtti;
using namespace mrpt::serialization;
using namespace mrpt::io;

// CStream
void CStream_ReadObject(CArchive& self, CSerializable::Ptr& obj)
{
	self.ReadObject(obj.get());
}

void CStream_WriteObject(CArchive& self, CSerializable::Ptr& obj)
{
	self.WriteObject(obj.get());
}
// end of CStream

// CConfigFileBase
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CConfigFileBase_read_double_overloads, read_double, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CConfigFileBase_read_float_overloads, read_float, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CConfigFileBase_read_bool_overloads, read_bool, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CConfigFileBase_read_int_overloads, read_int, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CConfigFileBase_read_uint64_t_overloads, read_uint64_t, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CConfigFileBase_read_string_overloads, read_string, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(
	CConfigFileBase_read_string_first_word_overloads, read_string_first_word, 3,
	4)
// TODO implement read_vector, read_matrix and read_enum
// end of CConfigFileBase

// Utils
double mrpt_utils_DEG2RAD(double deg) { return mrpt::DEG2RAD(deg); }
double mrpt_utils_RAD2DEG(double rad) { return mrpt::RAD2DEG(rad); }
// end of Utils

// smart pointer contents
CObject& CObjectPtr_get_ctx(CObject::Ptr& self) { return *self; }
void CObjectPtr_set_ctx(CObject::Ptr& self, const CObject& ctx) { *self = ctx; }
CObject* CObjectPtr_pointer(CObject::Ptr& self) { return self.get(); }
MAKE_PTR_CTX(CSerializable)

// exporter
void export_utils()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(utils)

	// TColorf
	{
		class_<TColorf>(
			"TColorf", init<optional<float, float, float, float>>(
						   args("r", "g", "b", "alpha"),
						   "A RGB color - floats in the range [0,1]."))
			.def_readwrite("R", &TColorf::R, "red")
			.def_readwrite("G", &TColorf::R, "green")
			.def_readwrite("B", &TColorf::R, "blue")
			.def_readwrite("A", &TColorf::R, "alpha");
	}

	// CObject
	{
		class_<CObject::Ptr>(
			"CObject::Ptr", "class_name smart pointer type", no_init)
			.def("ctx", &CObjectPtr_get_ctx, return_internal_reference<>())
			.def("ctx", &CObjectPtr_set_ctx)
			.def("pointer", &CObjectPtr_pointer, return_internal_reference<>());

		class_<CObject, boost::noncopyable>("CObject", no_init)
			.def(
				"clone", &CObject::clone,
				return_value_policy<manage_new_object>(),
				"Returns a copy of the object, indepently of its class.")
			.def(
				"GetRuntimeClass", &CObject::GetRuntimeClass,
				return_internal_reference<>(),
				"Returns information about the class of an object in runtime.");
	}

	// CLoadableOptions
	{
		class_<CLoadableOptions, boost::noncopyable>(
			"CLoadableOptions", no_init)
			.def(
				"loadFromConfigFile", &CLoadableOptions::loadFromConfigFile,
				"This method load the options from a \".ini\"-like file or "
				"memory-stored string list.")
			.def(
				"loadFromConfigFileName",
				&CLoadableOptions::loadFromConfigFileName,
				"Behaves like loadFromConfigFile, but you can pass directly a "
				"file name and a temporary CConfigFile object will be created "
				"automatically to load the file.")
			.def(
				"saveToConfigFile", &CLoadableOptions::saveToConfigFile,
				"This method saves the options to a \".ini\"-like file or "
				"memory-stored string list.")
			.def(
				"saveToConfigFileName", &CLoadableOptions::saveToConfigFileName,
				"Behaves like saveToConfigFile, but you can pass directly a "
				"file name and a temporary CConfigFile object will be created "
				"automatically to save the file.")
			.def(
				"dumpToConsole", &CLoadableOptions::dumpToConsole,
				"Just like dumpToTextStream() but sending the text to the "
				"console (std::cout)")
			.def(
				"dumpToTextStream", &CLoadableOptions::dumpToTextStream,
				"This method should clearly display all the contents of the "
				"structure in textual form, sending it to a CStream.");
	}

	// TRuntimeClassId
	{
		class_<TRuntimeClassId, boost::noncopyable>("TRuntimeClassId", no_init)
			.def_readwrite("className", &TRuntimeClassId::className);
	}

	// CConfigFileBase
	{
		class_<CConfigFileBase, boost::noncopyable>("CConfigFileBase", no_init)
			.def(
				"getAllSections", &CConfigFileBase::getAllSections,
				"Returns a list with all the section names.")
			.def(
				"getAllKeys", &CConfigFileBase::getAllKeys,
				"Returs a list with all the keys into a section.")
			.def(
				"sectionExists ", &CConfigFileBase::sectionExists,
				"Checks if a given section exists (name is case insensitive).")
			.def(
				"read_double", &CConfigFileBase::read_double,
				CConfigFileBase_read_double_overloads())
			.def(
				"read_float", &CConfigFileBase::read_float,
				CConfigFileBase_read_float_overloads())
			.def(
				"read_bool", &CConfigFileBase::read_bool,
				CConfigFileBase_read_bool_overloads())
			.def(
				"read_int", &CConfigFileBase::read_int,
				CConfigFileBase_read_int_overloads())
			.def(
				"read_uint64_t", &CConfigFileBase::read_uint64_t,
				CConfigFileBase_read_uint64_t_overloads())
			.def(
				"read_string", &CConfigFileBase::read_string,
				CConfigFileBase_read_string_overloads())
			.def(
				"read_string_first_word",
				&CConfigFileBase::read_string_first_word,
				CConfigFileBase_read_string_first_word_overloads());
	}

	// CConfigFile
	{
		class_<CConfigFile, bases<CConfigFileBase>>(
			"CConfigFile", init<optional<std::string>>());
	}

	// CSerializable
	{
		class_<CSerializable::Ptr>(
			"CSerializable::Ptr", "class_name smart pointer type", no_init)
			.def(
				"ctx", &CSerializablePtr_get_ctx, return_internal_reference<>())
			.def("ctx", &CSerializablePtr_set_ctx)
			.def(
				"pointer", &CSerializablePtr_pointer,
				return_internal_reference<>());

		class_<CSerializable, boost::noncopyable, bases<CObject>>(
			"CSerializable", no_init);
	}

	// CStream
	{
		class_<CStream, boost::noncopyable>(
			"CStream",
			"This base class is used to provide a unified interface to "
			"files,memory buffers.",
			no_init)
			.def(
				"ReadObject", &CStream_ReadObject,
				"Reads an object from stream, its class determined at runtime, "
				"and returns a smart pointer to the object.")
			.def(
				"WriteObject", &CStream_WriteObject,
				"Writes an object to the stream.");
	}

	// CFileGZInputStream
	{
		class_<CFileGZInputStream, boost::noncopyable, bases<CStream>>(
			"CFileGZInputStream",
			"Transparently opens a compressed \"gz\" file and reads "
			"uncompressed data from it.",
			init<optional<std::string>>(args("filename")))
			.def("open", &CFileGZInputStream::open, "Opens the file for read.")
			.def("close", &CFileGZInputStream::close, "Closes the file.");
	}

	// static module functions
	def("DEG2RAD", &mrpt_utils_DEG2RAD, args("deg"),
		"Convert degrees to radiants.");
	def("RAD2DEG", &mrpt_utils_RAD2DEG, args("rad"),
		"Convert radiants to degrees.");
}

void export_utils_stl()
{
	MAKE_VEC_NAMED(float, FloatVector)
	MAKE_VEC_NAMED(double, DoubleVector)
	MAKE_VEC_NAMED(uint8_t, UInt8Vector)
	MAKE_VEC_NAMED(uint16_t, UInt16Vector)
	MAKE_VEC_NAMED(uint32_t, UInt32Vector)
	MAKE_VEC_NAMED(uint64_t, UInt64Vector)
	MAKE_VEC_NAMED(int8_t, Int8Vector)
	MAKE_VEC_NAMED(int16_t, Int16Vector)
	MAKE_VEC_NAMED(int32_t, Int32Vector)
	MAKE_VEC_NAMED(int64_t, Int64Vector)
}
