#include <iterator>
#include <memory>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/open_flags.h>
#include <optional>
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

// mrpt::io::CStream file:mrpt/io/CStream.h line:29
struct PyCallBack_mrpt_io_CStream : public mrpt::io::CStream {
	using mrpt::io::CStream::CStream;

	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CStream *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CStream::Read\"");
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CStream *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CStream::Write\"");
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CStream *>(this), "ReadBufferImmediate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CStream::ReadBufferImmediate(a0, a1);
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CStream *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CStream::Seek\"");
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CStream *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CStream::getTotalBytesCount\"");
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CStream *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CStream::getPosition\"");
	}
	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CStream *>(this), "getStreamDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CStream::getStreamDescription();
	}
};

// mrpt::io::CFileGZInputStream file:mrpt/io/CFileGZInputStream.h line:24
struct PyCallBack_mrpt_io_CFileGZInputStream : public mrpt::io::CFileGZInputStream {
	using mrpt::io::CFileGZInputStream::CFileGZInputStream;

	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZInputStream *>(this), "getStreamDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CFileGZInputStream::getStreamDescription();
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZInputStream *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileGZInputStream::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZInputStream *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileGZInputStream::getPosition();
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZInputStream *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileGZInputStream::Seek(a0, a1);
	}
	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZInputStream *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileGZInputStream::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZInputStream *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileGZInputStream::Write(a0, a1);
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZInputStream *>(this), "ReadBufferImmediate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CStream::ReadBufferImmediate(a0, a1);
	}
};

// mrpt::io::CFileGZOutputStream file:mrpt/io/CFileGZOutputStream.h line:25
struct PyCallBack_mrpt_io_CFileGZOutputStream : public mrpt::io::CFileGZOutputStream {
	using mrpt::io::CFileGZOutputStream::CFileGZOutputStream;

	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZOutputStream *>(this), "getStreamDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CFileGZOutputStream::getStreamDescription();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZOutputStream *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileGZOutputStream::getPosition();
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZOutputStream *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileGZOutputStream::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZOutputStream *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileGZOutputStream::getTotalBytesCount();
	}
	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZOutputStream *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileGZOutputStream::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZOutputStream *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileGZOutputStream::Write(a0, a1);
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileGZOutputStream *>(this), "ReadBufferImmediate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CStream::ReadBufferImmediate(a0, a1);
	}
};

void bind_mrpt_io_CStream(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::io::CStream file:mrpt/io/CStream.h line:29
		pybind11::class_<mrpt::io::CStream, std::shared_ptr<mrpt::io::CStream>, PyCallBack_mrpt_io_CStream> cl(M("mrpt::io"), "CStream", "This base class is used to provide a unified interface to\n    files,memory buffers,..Please see the derived classes. This class is\n    largely inspired by Borland VCL \"TStream\" class. \n  Apart of the \"VCL like\" methods, operators \">>\" and \"<<\" have been\n    defined so that simple types (int,bool,char,float,char *,std::string,...)\n    can be directly written and read to and from any CStream easily.\n  Please, it is recomendable to read CSerializable documentation also.\n\n \n\n \n CFileStream, CMemoryStream,CSerializable");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_io_CStream(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_io_CStream const &>());

		pybind11::enum_<mrpt::io::CStream::TSeekOrigin>(cl, "TSeekOrigin", pybind11::arithmetic(), "Used in CStream::Seek ")
			.value("sFromBeginning", mrpt::io::CStream::sFromBeginning)
			.value("sFromCurrent", mrpt::io::CStream::sFromCurrent)
			.value("sFromEnd", mrpt::io::CStream::sFromEnd)
			.export_values();

		cl.def("Read", (size_t (mrpt::io::CStream::*)(void *, size_t)) &mrpt::io::CStream::Read, "Introduces a pure virtual method responsible for reading from the\n stream. \n\nC++: mrpt::io::CStream::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Write", (size_t (mrpt::io::CStream::*)(const void *, size_t)) &mrpt::io::CStream::Write, "Introduces a pure virtual method responsible for writing to the stream.\n  Write attempts to write up to Count bytes to Buffer, and returns the\n number of bytes actually written. \n\nC++: mrpt::io::CStream::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("ReadBufferImmediate", (size_t (mrpt::io::CStream::*)(void *, size_t)) &mrpt::io::CStream::ReadBufferImmediate, "Reads a block of bytes from the stream into Buffer, and returns the\namound of bytes actually read, without waiting for more extra bytes to\narrive (just those already enqued in the stream).\n  Note that this method will fallback to ReadBuffer() in most CStream\nclasses but in some hardware-related  classes.\n	\n\n std::exception On any error, or if ZERO bytes are read.\n\nC++: mrpt::io::CStream::ReadBufferImmediate(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Seek", [](mrpt::io::CStream &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg("Offset"));
		cl.def("Seek", (uint64_t (mrpt::io::CStream::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::io::CStream::Seek, "Introduces a pure virtual method for moving to a specified position in\nthe streamed resource.\n   he Origin parameter indicates how to interpret the Offset parameter.\nOrigin should be one of the following values:\n	- sFromBeginning	(Default) Offset is from the beginning of the\nresource. Seek moves to the position Offset. Offset must be >= 0.\n	- sFromCurrent		Offset is from the current position in the resource.\nSeek moves to Position + Offset.\n	- sFromEnd			Offset is from the end of the resource. Offset must\nbe\n<= 0 to indicate a number of bytes before the end of the file.\n \n\n Seek returns the new value of the Position property.\n\nC++: mrpt::io::CStream::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg("Offset"), pybind11::arg("Origin"));
		cl.def("getTotalBytesCount", (uint64_t (mrpt::io::CStream::*)() const) &mrpt::io::CStream::getTotalBytesCount, "Returns the total amount of bytes in the stream.\n\nC++: mrpt::io::CStream::getTotalBytesCount() const --> uint64_t");
		cl.def("getPosition", (uint64_t (mrpt::io::CStream::*)() const) &mrpt::io::CStream::getPosition, "Method for getting the current cursor position, where 0 is the first\n byte and TotalBytesCount-1 the last one.\n\nC++: mrpt::io::CStream::getPosition() const --> uint64_t");
		cl.def("getline", (bool (mrpt::io::CStream::*)(std::string &)) &mrpt::io::CStream::getline, "Reads from the stream until a `` character is found (`` characters\n are ignored).\n \n\n false on EOF or any other read error.\n\nC++: mrpt::io::CStream::getline(std::string &) --> bool", pybind11::arg("out_str"));
		cl.def("getStreamDescription", (std::string (mrpt::io::CStream::*)() const) &mrpt::io::CStream::getStreamDescription, "Returns a human-friendly description of the stream, e.g. a filename. \n\nC++: mrpt::io::CStream::getStreamDescription() const --> std::string");
		cl.def("assign", (class mrpt::io::CStream & (mrpt::io::CStream::*)(const class mrpt::io::CStream &)) &mrpt::io::CStream::operator=, "C++: mrpt::io::CStream::operator=(const class mrpt::io::CStream &) --> class mrpt::io::CStream &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::io::CFileGZInputStream file:mrpt/io/CFileGZInputStream.h line:24
		pybind11::class_<mrpt::io::CFileGZInputStream, std::shared_ptr<mrpt::io::CFileGZInputStream>, PyCallBack_mrpt_io_CFileGZInputStream, mrpt::io::CStream> cl(M("mrpt::io"), "CFileGZInputStream", "Transparently opens a compressed \"gz\" file and reads uncompressed data from\n it.\n If the file is not a .gz file, it silently reads data from the file.\n\n \n CFileInputStream\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::io::CFileGZInputStream(); }, [](){ return new PyCallBack_mrpt_io_CFileGZInputStream(); } ) );
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("fileName") );

		cl.def("getStreamDescription", (std::string (mrpt::io::CFileGZInputStream::*)() const) &mrpt::io::CFileGZInputStream::getStreamDescription, "C++: mrpt::io::CFileGZInputStream::getStreamDescription() const --> std::string");
		cl.def("close", (void (mrpt::io::CFileGZInputStream::*)()) &mrpt::io::CFileGZInputStream::close, "Closes the file \n\nC++: mrpt::io::CFileGZInputStream::close() --> void");
		cl.def("fileOpenCorrectly", (bool (mrpt::io::CFileGZInputStream::*)() const) &mrpt::io::CFileGZInputStream::fileOpenCorrectly, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileGZInputStream::fileOpenCorrectly() const --> bool");
		cl.def("is_open", (bool (mrpt::io::CFileGZInputStream::*)()) &mrpt::io::CFileGZInputStream::is_open, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileGZInputStream::is_open() --> bool");
		cl.def("checkEOF", (bool (mrpt::io::CFileGZInputStream::*)()) &mrpt::io::CFileGZInputStream::checkEOF, "Will be true if EOF has been already reached. \n\nC++: mrpt::io::CFileGZInputStream::checkEOF() --> bool");
		cl.def("filePathAtUse", (std::string (mrpt::io::CFileGZInputStream::*)() const) &mrpt::io::CFileGZInputStream::filePathAtUse, "Returns the path of the filename passed to open(), or empty if none. \n\nC++: mrpt::io::CFileGZInputStream::filePathAtUse() const --> std::string");
		cl.def("getTotalBytesCount", (uint64_t (mrpt::io::CFileGZInputStream::*)() const) &mrpt::io::CFileGZInputStream::getTotalBytesCount, "Method for getting the total number of compressed bytes of in the\n file (the physical size of the compressed file). \n\nC++: mrpt::io::CFileGZInputStream::getTotalBytesCount() const --> uint64_t");
		cl.def("getPosition", (uint64_t (mrpt::io::CFileGZInputStream::*)() const) &mrpt::io::CFileGZInputStream::getPosition, "Method for getting the current cursor position in the compressed,\n where 0 is the first byte and TotalBytesCount-1 the last one. \n\nC++: mrpt::io::CFileGZInputStream::getPosition() const --> uint64_t");
		cl.def("Seek", [](mrpt::io::CFileGZInputStream &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg(""));
		cl.def("Seek", (uint64_t (mrpt::io::CFileGZInputStream::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::io::CFileGZInputStream::Seek, "This method is not implemented in this class \n\nC++: mrpt::io::CFileGZInputStream::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg(""), pybind11::arg(""));
		cl.def("Read", (size_t (mrpt::io::CFileGZInputStream::*)(void *, size_t)) &mrpt::io::CFileGZInputStream::Read, "C++: mrpt::io::CFileGZInputStream::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Write", (size_t (mrpt::io::CFileGZInputStream::*)(const void *, size_t)) &mrpt::io::CFileGZInputStream::Write, "C++: mrpt::io::CFileGZInputStream::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
	}
	// mrpt::io::OpenMode file:mrpt/io/open_flags.h line:20
	pybind11::enum_<mrpt::io::OpenMode>(M("mrpt::io"), "OpenMode", "Flags used when opening a file for writing.\n\n \n CFileGZOutputStream, CFileGZOutputStream\n \n\n\n ")
		.value("TRUNCATE", mrpt::io::OpenMode::TRUNCATE)
		.value("APPEND", mrpt::io::OpenMode::APPEND);

;

	{ // mrpt::io::CFileGZOutputStream file:mrpt/io/CFileGZOutputStream.h line:25
		pybind11::class_<mrpt::io::CFileGZOutputStream, std::shared_ptr<mrpt::io::CFileGZOutputStream>, PyCallBack_mrpt_io_CFileGZOutputStream, mrpt::io::CStream> cl(M("mrpt::io"), "CFileGZOutputStream", "Saves data to a file and transparently compress the data using the given\n compression level.\n The generated files are in gzip format (\"file.gz\").\n\n \n CFileOutputStream\n \n\n\n ");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::io::CFileGZOutputStream(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_io_CFileGZOutputStream(a0); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, const enum mrpt::io::OpenMode & a1){ return new mrpt::io::CFileGZOutputStream(a0, a1); }, [](const std::string & a0, const enum mrpt::io::OpenMode & a1){ return new PyCallBack_mrpt_io_CFileGZOutputStream(a0, a1); } ), "doc");
		cl.def( pybind11::init<const std::string &, const enum mrpt::io::OpenMode, int>(), pybind11::arg("fileName"), pybind11::arg("mode"), pybind11::arg("compressionLevel") );

		cl.def( pybind11::init( [](){ return new mrpt::io::CFileGZOutputStream(); }, [](){ return new PyCallBack_mrpt_io_CFileGZOutputStream(); } ) );
		cl.def("getStreamDescription", (std::string (mrpt::io::CFileGZOutputStream::*)() const) &mrpt::io::CFileGZOutputStream::getStreamDescription, "C++: mrpt::io::CFileGZOutputStream::getStreamDescription() const --> std::string");
		cl.def("close", (void (mrpt::io::CFileGZOutputStream::*)()) &mrpt::io::CFileGZOutputStream::close, "Close the file \n\nC++: mrpt::io::CFileGZOutputStream::close() --> void");
		cl.def("fileOpenCorrectly", (bool (mrpt::io::CFileGZOutputStream::*)() const) &mrpt::io::CFileGZOutputStream::fileOpenCorrectly, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileGZOutputStream::fileOpenCorrectly() const --> bool");
		cl.def("is_open", (bool (mrpt::io::CFileGZOutputStream::*)()) &mrpt::io::CFileGZOutputStream::is_open, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileGZOutputStream::is_open() --> bool");
		cl.def("getPosition", (uint64_t (mrpt::io::CFileGZOutputStream::*)() const) &mrpt::io::CFileGZOutputStream::getPosition, "Method for getting the current cursor position, where 0 is the first\n byte and TotalBytesCount-1 the last one. \n\nC++: mrpt::io::CFileGZOutputStream::getPosition() const --> uint64_t");
		cl.def("filePathAtUse", (std::string (mrpt::io::CFileGZOutputStream::*)() const) &mrpt::io::CFileGZOutputStream::filePathAtUse, "Returns the path of the filename passed to open(), or empty if none. \n\nC++: mrpt::io::CFileGZOutputStream::filePathAtUse() const --> std::string");
		cl.def("Seek", [](mrpt::io::CFileGZOutputStream &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg(""));
		cl.def("Seek", (uint64_t (mrpt::io::CFileGZOutputStream::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::io::CFileGZOutputStream::Seek, "This method is not implemented in this class \n\nC++: mrpt::io::CFileGZOutputStream::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg(""), pybind11::arg(""));
		cl.def("getTotalBytesCount", (uint64_t (mrpt::io::CFileGZOutputStream::*)() const) &mrpt::io::CFileGZOutputStream::getTotalBytesCount, "This method is not implemented in this class \n\nC++: mrpt::io::CFileGZOutputStream::getTotalBytesCount() const --> uint64_t");
		cl.def("Read", (size_t (mrpt::io::CFileGZOutputStream::*)(void *, size_t)) &mrpt::io::CFileGZOutputStream::Read, "C++: mrpt::io::CFileGZOutputStream::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Write", (size_t (mrpt::io::CFileGZOutputStream::*)(const void *, size_t)) &mrpt::io::CFileGZOutputStream::Write, "C++: mrpt::io::CFileGZOutputStream::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
	}
}
