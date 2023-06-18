#include <iterator>
#include <memory>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <sstream> // __str__
#include <string>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_serialization_CMessage(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::serialization::CMessage file:mrpt/serialization/CMessage.h line:28
		pybind11::class_<mrpt::serialization::CMessage, std::shared_ptr<mrpt::serialization::CMessage>> cl(M("mrpt::serialization"), "CMessage", "A class that contain generic messages, that can be sent and received from a\n \"CClientTCPSocket\" object.\n  A message consists of a \"header\" (or type), and a \"body\" (or content).\n  Apart from arbitrary data, specific methods are provided for easing the\n serialization of MRPT's \"CSerializable\" objects.\n  This class is also used for passing data to hardware interfaces (see\n mrpt::comms::CSerialPort)\n \n\n CClientTCPSocket\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::serialization::CMessage(); } ) );
		cl.def( pybind11::init( [](mrpt::serialization::CMessage const &o){ return new mrpt::serialization::CMessage(o); } ) );
		cl.def_readwrite("type", &mrpt::serialization::CMessage::type);
		cl.def_readwrite("content", &mrpt::serialization::CMessage::content);
		cl.def("serializeObject", (void (mrpt::serialization::CMessage::*)(const class mrpt::serialization::CSerializable *)) &mrpt::serialization::CMessage::serializeObject, "A method for serializing a MRPT's object into the content.\n  Any modification to data in \"content\" after this will corrupt the\n object serialization.\n  Member \"type\" is unmodified in this method.\n\nC++: mrpt::serialization::CMessage::serializeObject(const class mrpt::serialization::CSerializable *) --> void", pybind11::arg("obj"));
		cl.def("deserializeIntoExistingObject", (void (mrpt::serialization::CMessage::*)(class mrpt::serialization::CSerializable *)) &mrpt::serialization::CMessage::deserializeIntoExistingObject, "A method that parse the data in the message into an existing object.\n  Note that the class of the object must be known and must match the one\n of the serialized object.\n  std::exception On corrupt data, unknown serialized objects,\n unknown serialized object version, non-matching classes,...\n\nC++: mrpt::serialization::CMessage::deserializeIntoExistingObject(class mrpt::serialization::CSerializable *) --> void", pybind11::arg("obj"));
		cl.def("deserializeIntoNewObject", (void (mrpt::serialization::CMessage::*)(class std::shared_ptr<class mrpt::serialization::CSerializable> &)) &mrpt::serialization::CMessage::deserializeIntoNewObject, "A method that parse the data in the message into a new object of (a\n priori) unknown class.\n  The pointer will contain on return a copy of the reconstructed object.\n Deleting this object when\n   no longer required is the responsability of the user. Note that\n previous contents of the pointer\n   will be ignored (it should be nullptr).\n  std::exception On corrupt data, unknown serialized objects,\n unknown serialized object version,...\n\nC++: mrpt::serialization::CMessage::deserializeIntoNewObject(class std::shared_ptr<class mrpt::serialization::CSerializable> &) --> void", pybind11::arg("obj"));
		cl.def("setContentFromString", (void (mrpt::serialization::CMessage::*)(const std::string &)) &mrpt::serialization::CMessage::setContentFromString, "Sets the contents of the message from a string\n \n\n getContentAsString\n\nC++: mrpt::serialization::CMessage::setContentFromString(const std::string &) --> void", pybind11::arg("str"));
		cl.def("getContentAsString", (void (mrpt::serialization::CMessage::*)(std::string &)) &mrpt::serialization::CMessage::getContentAsString, "Gets the contents of the message as a string\n \n\n setContentFromString\n\nC++: mrpt::serialization::CMessage::getContentAsString(std::string &) --> void", pybind11::arg("str"));
		cl.def("assign", (class mrpt::serialization::CMessage & (mrpt::serialization::CMessage::*)(const class mrpt::serialization::CMessage &)) &mrpt::serialization::CMessage::operator=, "C++: mrpt::serialization::CMessage::operator=(const class mrpt::serialization::CMessage &) --> class mrpt::serialization::CMessage &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
