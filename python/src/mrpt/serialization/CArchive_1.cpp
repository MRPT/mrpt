#include <iterator>
#include <memory>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/open_flags.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
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

// mrpt::serialization::CArchiveStreamBase file:mrpt/serialization/CArchive.h line:602
struct PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileInputStream_t : public mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream> {
	using mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream>::CArchiveStreamBase;

	std::string getArchiveDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream> *>(this), "getArchiveDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CArchiveStreamBase::getArchiveDescription();
	}
	size_t write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream> *>(this), "write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CArchiveStreamBase::write(a0, a1);
	}
	size_t read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream> *>(this), "read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CArchiveStreamBase::read(a0, a1);
	}
};

// mrpt::serialization::CArchiveStreamBase file:mrpt/serialization/CArchive.h line:602
struct PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileOutputStream_t : public mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream> {
	using mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream>::CArchiveStreamBase;

	std::string getArchiveDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream> *>(this), "getArchiveDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CArchiveStreamBase::getArchiveDescription();
	}
	size_t write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream> *>(this), "write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CArchiveStreamBase::write(a0, a1);
	}
	size_t read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream> *>(this), "read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CArchiveStreamBase::read(a0, a1);
	}
};

void bind_mrpt_serialization_CArchive_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::serialization::CArchiveStreamBase file:mrpt/serialization/CArchive.h line:602
		pybind11::class_<mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream>, std::shared_ptr<mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream>>, PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileInputStream_t, mrpt::serialization::CArchive> cl(M("mrpt::serialization"), "CArchiveStreamBase_mrpt_io_CFileInputStream_t", "");
		cl.def( pybind11::init<class mrpt::io::CFileInputStream &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileInputStream_t const &o){ return new PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileInputStream_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream> const &o){ return new mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream>(o); } ) );
		cl.def("getArchiveDescription", (std::string (mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream>::*)() const) &mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream>::getArchiveDescription, "C++: mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileInputStream>::getArchiveDescription() const --> std::string");
		cl.def("ReadAs", (unsigned int (mrpt::serialization::CArchive::*)()) &mrpt::serialization::CArchive::ReadAs<unsigned int>, "C++: mrpt::serialization::CArchive::ReadAs() --> unsigned int");
		cl.def("WriteAs", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(const unsigned long &)) &mrpt::serialization::CArchive::WriteAs<unsigned int,unsigned long>, "C++: mrpt::serialization::CArchive::WriteAs(const unsigned long &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("value"));
		cl.def("ReadObject", (class std::shared_ptr<class mrpt::serialization::CSerializable> (mrpt::serialization::CArchive::*)()) &mrpt::serialization::CArchive::ReadObject<mrpt::serialization::CSerializable>, "C++: mrpt::serialization::CArchive::ReadObject() --> class std::shared_ptr<class mrpt::serialization::CSerializable>");
		cl.def("ReadBuffer", (size_t (mrpt::serialization::CArchive::*)(void *, size_t)) &mrpt::serialization::CArchive::ReadBuffer, "@{ \n\n Reads a block of bytes from the stream into Buffer\n	\n\n std::exception On any error, or if ZERO bytes are read.\n  \n\n The amound of bytes actually read.\n \n\n This method is endianness-dependent.\n \n\n ReadBufferImmediate ; Important, see: ReadBufferFixEndianness,\n\nC++: mrpt::serialization::CArchive::ReadBuffer(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("WriteBuffer", (void (mrpt::serialization::CArchive::*)(const void *, size_t)) &mrpt::serialization::CArchive::WriteBuffer, "Writes a block of bytes to the stream from Buffer.\n	\n\n std::exception On any error\n  \n\n Important, see: WriteBufferFixEndianness\n \n\n This method is endianness-dependent.\n\nC++: mrpt::serialization::CArchive::WriteBuffer(const void *, size_t) --> void", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("WriteObject", (void (mrpt::serialization::CArchive::*)(const class mrpt::serialization::CSerializable *)) &mrpt::serialization::CArchive::WriteObject, "Writes an object to the stream.\n\nC++: mrpt::serialization::CArchive::WriteObject(const class mrpt::serialization::CSerializable *) --> void", pybind11::arg("o"));
		cl.def("WriteObject", (void (mrpt::serialization::CArchive::*)(const class mrpt::serialization::CSerializable &)) &mrpt::serialization::CArchive::WriteObject, "C++: mrpt::serialization::CArchive::WriteObject(const class mrpt::serialization::CSerializable &) --> void", pybind11::arg("o"));
		cl.def("ReadObject", (class std::shared_ptr<class mrpt::serialization::CSerializable> (mrpt::serialization::CArchive::*)()) &mrpt::serialization::CArchive::ReadObject, "Reads an object from stream, its class determined at runtime, and\n returns a smart pointer to the object.\n \n\n std::exception On I/O error or undefined class.\n \n\n CExceptionEOF On an End-Of-File condition found\n at a correct place: an EOF that abruptly finishes in the middle of one\n object raises a plain std::exception instead.\n\nC++: mrpt::serialization::CArchive::ReadObject() --> class std::shared_ptr<class mrpt::serialization::CSerializable>");
		cl.def("getArchiveDescription", (std::string (mrpt::serialization::CArchive::*)() const) &mrpt::serialization::CArchive::getArchiveDescription, "If redefined in derived classes, allows finding a human-friendly\n description of the underlying stream (e.g. filename) \n\nC++: mrpt::serialization::CArchive::getArchiveDescription() const --> std::string");
		cl.def("ReadObject", (void (mrpt::serialization::CArchive::*)(class mrpt::serialization::CSerializable *)) &mrpt::serialization::CArchive::ReadObject, "Reads an object from stream, where its class must be the same\n    as the supplied object, where the loaded object will be stored in.\n \n\n std::exception On I/O error or different class found.\n \n\n CExceptionEOF On an End-Of-File condition found\n at a correct place: an EOF that abruptly finishes in the middle of one\n object raises a plain std::exception instead.\n\nC++: mrpt::serialization::CArchive::ReadObject(class mrpt::serialization::CSerializable *) --> void", pybind11::arg("existingObj"));
		cl.def("__lshift__", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(const class mrpt::serialization::CSerializable &)) &mrpt::serialization::CArchive::operator<<, "Write a CSerializable object to a stream in the binary MRPT format \n\nC++: mrpt::serialization::CArchive::operator<<(const class mrpt::serialization::CSerializable &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("obj"));
		cl.def("__lshift__", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(const class std::shared_ptr<class mrpt::serialization::CSerializable> &)) &mrpt::serialization::CArchive::operator<<, "C++: mrpt::serialization::CArchive::operator<<(const class std::shared_ptr<class mrpt::serialization::CSerializable> &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("pObj"));
		cl.def("__rshift__", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(class mrpt::serialization::CSerializable &)) &mrpt::serialization::CArchive::operator>>, "Reads a CSerializable object from the stream \n\nC++: mrpt::serialization::CArchive::operator>>(class mrpt::serialization::CSerializable &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("obj"));
		cl.def("__rshift__", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(class std::shared_ptr<class mrpt::serialization::CSerializable> &)) &mrpt::serialization::CArchive::operator>>, "C++: mrpt::serialization::CArchive::operator>>(class std::shared_ptr<class mrpt::serialization::CSerializable> &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("pObj"));
		cl.def("assign", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(const class mrpt::serialization::CArchive &)) &mrpt::serialization::CArchive::operator=, "C++: mrpt::serialization::CArchive::operator=(const class mrpt::serialization::CArchive &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::serialization::CArchiveStreamBase file:mrpt/serialization/CArchive.h line:602
		pybind11::class_<mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream>, std::shared_ptr<mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream>>, PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileOutputStream_t, mrpt::serialization::CArchive> cl(M("mrpt::serialization"), "CArchiveStreamBase_mrpt_io_CFileOutputStream_t", "");
		cl.def( pybind11::init<class mrpt::io::CFileOutputStream &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileOutputStream_t const &o){ return new PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileOutputStream_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream> const &o){ return new mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream>(o); } ) );
		cl.def("getArchiveDescription", (std::string (mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream>::*)() const) &mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream>::getArchiveDescription, "C++: mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileOutputStream>::getArchiveDescription() const --> std::string");
		cl.def("ReadAs", (unsigned int (mrpt::serialization::CArchive::*)()) &mrpt::serialization::CArchive::ReadAs<unsigned int>, "C++: mrpt::serialization::CArchive::ReadAs() --> unsigned int");
		cl.def("WriteAs", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(const unsigned long &)) &mrpt::serialization::CArchive::WriteAs<unsigned int,unsigned long>, "C++: mrpt::serialization::CArchive::WriteAs(const unsigned long &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("value"));
		cl.def("ReadObject", (class std::shared_ptr<class mrpt::serialization::CSerializable> (mrpt::serialization::CArchive::*)()) &mrpt::serialization::CArchive::ReadObject<mrpt::serialization::CSerializable>, "C++: mrpt::serialization::CArchive::ReadObject() --> class std::shared_ptr<class mrpt::serialization::CSerializable>");
		cl.def("ReadBuffer", (size_t (mrpt::serialization::CArchive::*)(void *, size_t)) &mrpt::serialization::CArchive::ReadBuffer, "@{ \n\n Reads a block of bytes from the stream into Buffer\n	\n\n std::exception On any error, or if ZERO bytes are read.\n  \n\n The amound of bytes actually read.\n \n\n This method is endianness-dependent.\n \n\n ReadBufferImmediate ; Important, see: ReadBufferFixEndianness,\n\nC++: mrpt::serialization::CArchive::ReadBuffer(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("WriteBuffer", (void (mrpt::serialization::CArchive::*)(const void *, size_t)) &mrpt::serialization::CArchive::WriteBuffer, "Writes a block of bytes to the stream from Buffer.\n	\n\n std::exception On any error\n  \n\n Important, see: WriteBufferFixEndianness\n \n\n This method is endianness-dependent.\n\nC++: mrpt::serialization::CArchive::WriteBuffer(const void *, size_t) --> void", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("WriteObject", (void (mrpt::serialization::CArchive::*)(const class mrpt::serialization::CSerializable *)) &mrpt::serialization::CArchive::WriteObject, "Writes an object to the stream.\n\nC++: mrpt::serialization::CArchive::WriteObject(const class mrpt::serialization::CSerializable *) --> void", pybind11::arg("o"));
		cl.def("WriteObject", (void (mrpt::serialization::CArchive::*)(const class mrpt::serialization::CSerializable &)) &mrpt::serialization::CArchive::WriteObject, "C++: mrpt::serialization::CArchive::WriteObject(const class mrpt::serialization::CSerializable &) --> void", pybind11::arg("o"));
		cl.def("ReadObject", (class std::shared_ptr<class mrpt::serialization::CSerializable> (mrpt::serialization::CArchive::*)()) &mrpt::serialization::CArchive::ReadObject, "Reads an object from stream, its class determined at runtime, and\n returns a smart pointer to the object.\n \n\n std::exception On I/O error or undefined class.\n \n\n CExceptionEOF On an End-Of-File condition found\n at a correct place: an EOF that abruptly finishes in the middle of one\n object raises a plain std::exception instead.\n\nC++: mrpt::serialization::CArchive::ReadObject() --> class std::shared_ptr<class mrpt::serialization::CSerializable>");
		cl.def("getArchiveDescription", (std::string (mrpt::serialization::CArchive::*)() const) &mrpt::serialization::CArchive::getArchiveDescription, "If redefined in derived classes, allows finding a human-friendly\n description of the underlying stream (e.g. filename) \n\nC++: mrpt::serialization::CArchive::getArchiveDescription() const --> std::string");
		cl.def("ReadObject", (void (mrpt::serialization::CArchive::*)(class mrpt::serialization::CSerializable *)) &mrpt::serialization::CArchive::ReadObject, "Reads an object from stream, where its class must be the same\n    as the supplied object, where the loaded object will be stored in.\n \n\n std::exception On I/O error or different class found.\n \n\n CExceptionEOF On an End-Of-File condition found\n at a correct place: an EOF that abruptly finishes in the middle of one\n object raises a plain std::exception instead.\n\nC++: mrpt::serialization::CArchive::ReadObject(class mrpt::serialization::CSerializable *) --> void", pybind11::arg("existingObj"));
		cl.def("__lshift__", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(const class mrpt::serialization::CSerializable &)) &mrpt::serialization::CArchive::operator<<, "Write a CSerializable object to a stream in the binary MRPT format \n\nC++: mrpt::serialization::CArchive::operator<<(const class mrpt::serialization::CSerializable &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("obj"));
		cl.def("__lshift__", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(const class std::shared_ptr<class mrpt::serialization::CSerializable> &)) &mrpt::serialization::CArchive::operator<<, "C++: mrpt::serialization::CArchive::operator<<(const class std::shared_ptr<class mrpt::serialization::CSerializable> &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("pObj"));
		cl.def("__rshift__", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(class mrpt::serialization::CSerializable &)) &mrpt::serialization::CArchive::operator>>, "Reads a CSerializable object from the stream \n\nC++: mrpt::serialization::CArchive::operator>>(class mrpt::serialization::CSerializable &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("obj"));
		cl.def("__rshift__", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(class std::shared_ptr<class mrpt::serialization::CSerializable> &)) &mrpt::serialization::CArchive::operator>>, "C++: mrpt::serialization::CArchive::operator>>(class std::shared_ptr<class mrpt::serialization::CSerializable> &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg("pObj"));
		cl.def("assign", (class mrpt::serialization::CArchive & (mrpt::serialization::CArchive::*)(const class mrpt::serialization::CArchive &)) &mrpt::serialization::CArchive::operator=, "C++: mrpt::serialization::CArchive::operator=(const class mrpt::serialization::CArchive &) --> class mrpt::serialization::CArchive &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
