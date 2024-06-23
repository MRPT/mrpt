#include <iterator>
#include <memory>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/open_flags.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <optional>
#include <sstream> // __str__
#include <stdexcept>
#include <string>
#include <variant>

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

// mrpt::serialization::CExceptionEOF file:mrpt/serialization/CArchive.h line:38
struct PyCallBack_mrpt_serialization_CExceptionEOF : public mrpt::serialization::CExceptionEOF {
	using mrpt::serialization::CExceptionEOF::CExceptionEOF;

	const char * what() const throw() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CExceptionEOF *>(this), "what");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const char *>::value) {
				static pybind11::detail::override_caster_t<const char *> caster;
				return pybind11::detail::cast_ref<const char *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const char *>(std::move(o));
		}
		return runtime_error::what();
	}
};

// mrpt::serialization::CArchive file:mrpt/serialization/CArchive.h line:56
struct PyCallBack_mrpt_serialization_CArchive : public mrpt::serialization::CArchive {
	using mrpt::serialization::CArchive::CArchive;

	std::string getArchiveDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchive *>(this), "getArchiveDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CArchive::getArchiveDescription();
	}
	size_t write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchive *>(this), "write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CArchive::write\"");
	}
	size_t read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchive *>(this), "read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CArchive::read\"");
	}
};

// mrpt::serialization::CArchiveStreamBase file:mrpt/serialization/CArchive.h line:602
struct PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileGZInputStream_t : public mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream> {
	using mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream>::CArchiveStreamBase;

	std::string getArchiveDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream> *>(this), "getArchiveDescription");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream> *>(this), "write");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream> *>(this), "read");
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
struct PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileGZOutputStream_t : public mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream> {
	using mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream>::CArchiveStreamBase;

	std::string getArchiveDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream> *>(this), "getArchiveDescription");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream> *>(this), "write");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream> *>(this), "read");
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

void bind_mrpt_serialization_CArchive(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::serialization::CExceptionEOF file:mrpt/serialization/CArchive.h line:38
		pybind11::class_<mrpt::serialization::CExceptionEOF, std::shared_ptr<mrpt::serialization::CExceptionEOF>, PyCallBack_mrpt_serialization_CExceptionEOF> cl(M("mrpt::serialization"), "CExceptionEOF", "Used in mrpt::serialization::CArchive ");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_serialization_CExceptionEOF const &o){ return new PyCallBack_mrpt_serialization_CExceptionEOF(o); } ) );
		cl.def( pybind11::init( [](mrpt::serialization::CExceptionEOF const &o){ return new mrpt::serialization::CExceptionEOF(o); } ) );
		cl.def("assign", (class mrpt::serialization::CExceptionEOF & (mrpt::serialization::CExceptionEOF::*)(const class mrpt::serialization::CExceptionEOF &)) &mrpt::serialization::CExceptionEOF::operator=, "C++: mrpt::serialization::CExceptionEOF::operator=(const class mrpt::serialization::CExceptionEOF &) --> class mrpt::serialization::CExceptionEOF &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::serialization::CArchive file:mrpt/serialization/CArchive.h line:56
		pybind11::class_<mrpt::serialization::CArchive, std::shared_ptr<mrpt::serialization::CArchive>, PyCallBack_mrpt_serialization_CArchive> cl(M("mrpt::serialization"), "CArchive", "Virtual base class for \"archives\": classes abstracting I/O streams.\n This class separates the implementation details of serialization (in\n CSerializable) and data storage (CArchive children: files, sockets,...).\n\n Two main sets of implementations are provided:\n - archiveFrom: for MRPT mrpt::io::CArchive objects, and\n - CArchiveStdIStream and CArchiveStdOStream: for std::istream and\n std::ostream, respectively.\n\n \n mrpt::io::CArchive, mrpt::serialization::CSerializable\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_serialization_CArchive(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_serialization_CArchive const &>());
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
		pybind11::class_<mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream>, std::shared_ptr<mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream>>, PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileGZInputStream_t, mrpt::serialization::CArchive> cl(M("mrpt::serialization"), "CArchiveStreamBase_mrpt_io_CFileGZInputStream_t", "");
		cl.def( pybind11::init<class mrpt::io::CFileGZInputStream &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileGZInputStream_t const &o){ return new PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileGZInputStream_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream> const &o){ return new mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream>(o); } ) );
		cl.def("getArchiveDescription", (std::string (mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream>::*)() const) &mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream>::getArchiveDescription, "C++: mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZInputStream>::getArchiveDescription() const --> std::string");
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
		pybind11::class_<mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream>, std::shared_ptr<mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream>>, PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileGZOutputStream_t, mrpt::serialization::CArchive> cl(M("mrpt::serialization"), "CArchiveStreamBase_mrpt_io_CFileGZOutputStream_t", "");
		cl.def( pybind11::init<class mrpt::io::CFileGZOutputStream &>(), pybind11::arg("s") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileGZOutputStream_t const &o){ return new PyCallBack_mrpt_serialization_CArchiveStreamBase_mrpt_io_CFileGZOutputStream_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream> const &o){ return new mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream>(o); } ) );
		cl.def("getArchiveDescription", (std::string (mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream>::*)() const) &mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream>::getArchiveDescription, "C++: mrpt::serialization::CArchiveStreamBase<mrpt::io::CFileGZOutputStream>::getArchiveDescription() const --> std::string");
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
