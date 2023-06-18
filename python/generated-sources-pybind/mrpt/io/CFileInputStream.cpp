#include <iterator>
#include <memory>
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CStream.h>
#include <mrpt/io/open_flags.h>
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

// mrpt::io::CFileInputStream file:mrpt/io/CFileInputStream.h line:22
struct PyCallBack_mrpt_io_CFileInputStream : public mrpt::io::CFileInputStream {
	using mrpt::io::CFileInputStream::CFileInputStream;

	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileInputStream *>(this), "getStreamDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CFileInputStream::getStreamDescription();
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileInputStream *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileInputStream::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileInputStream *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileInputStream::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileInputStream *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileInputStream::getPosition();
	}
	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileInputStream *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileInputStream::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileInputStream *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileInputStream::Write(a0, a1);
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileInputStream *>(this), "ReadBufferImmediate");
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

// mrpt::io::CFileOutputStream file:mrpt/io/CFileOutputStream.h line:24
struct PyCallBack_mrpt_io_CFileOutputStream : public mrpt::io::CFileOutputStream {
	using mrpt::io::CFileOutputStream::CFileOutputStream;

	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileOutputStream *>(this), "getStreamDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CFileOutputStream::getStreamDescription();
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileOutputStream *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileOutputStream::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileOutputStream *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileOutputStream::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileOutputStream *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileOutputStream::getPosition();
	}
	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileOutputStream *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileOutputStream::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileOutputStream *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileOutputStream::Write(a0, a1);
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileOutputStream *>(this), "ReadBufferImmediate");
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

void bind_mrpt_io_CFileInputStream(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::io::CFileInputStream file:mrpt/io/CFileInputStream.h line:22
		pybind11::class_<mrpt::io::CFileInputStream, std::shared_ptr<mrpt::io::CFileInputStream>, PyCallBack_mrpt_io_CFileInputStream, mrpt::io::CStream> cl(M("mrpt::io"), "CFileInputStream", "This CStream derived class allow using a file as a read-only, binary stream.\n\n \n CStream, CFileStream, CFileGZInputStream\n \n\n\n ");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("fileName") );

		cl.def( pybind11::init( [](){ return new mrpt::io::CFileInputStream(); }, [](){ return new PyCallBack_mrpt_io_CFileInputStream(); } ) );
		cl.def("open", (bool (mrpt::io::CFileInputStream::*)(const std::string &)) &mrpt::io::CFileInputStream::open, "Open a file for reading\n \n\n The file to be open in this stream\n \n\n true on success.\n\nC++: mrpt::io::CFileInputStream::open(const std::string &) --> bool", pybind11::arg("fileName"));
		cl.def("close", (void (mrpt::io::CFileInputStream::*)()) &mrpt::io::CFileInputStream::close, "Close the stream \n\nC++: mrpt::io::CFileInputStream::close() --> void");
		cl.def("fileOpenCorrectly", (bool (mrpt::io::CFileInputStream::*)() const) &mrpt::io::CFileInputStream::fileOpenCorrectly, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileInputStream::fileOpenCorrectly() const --> bool");
		cl.def("is_open", (bool (mrpt::io::CFileInputStream::*)()) &mrpt::io::CFileInputStream::is_open, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileInputStream::is_open() --> bool");
		cl.def("checkEOF", (bool (mrpt::io::CFileInputStream::*)()) &mrpt::io::CFileInputStream::checkEOF, "Will be true if EOF has been already reached. \n\nC++: mrpt::io::CFileInputStream::checkEOF() --> bool");
		cl.def("clearError", (void (mrpt::io::CFileInputStream::*)()) &mrpt::io::CFileInputStream::clearError, "Resets stream error status bits (e.g. after an EOF) \n\nC++: mrpt::io::CFileInputStream::clearError() --> void");
		cl.def("getStreamDescription", (std::string (mrpt::io::CFileInputStream::*)() const) &mrpt::io::CFileInputStream::getStreamDescription, "C++: mrpt::io::CFileInputStream::getStreamDescription() const --> std::string");
		cl.def("Seek", [](mrpt::io::CFileInputStream &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg("off"));
		cl.def("Seek", (uint64_t (mrpt::io::CFileInputStream::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::io::CFileInputStream::Seek, "C++: mrpt::io::CFileInputStream::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg("off"), pybind11::arg("Origin"));
		cl.def("getTotalBytesCount", (uint64_t (mrpt::io::CFileInputStream::*)() const) &mrpt::io::CFileInputStream::getTotalBytesCount, "C++: mrpt::io::CFileInputStream::getTotalBytesCount() const --> uint64_t");
		cl.def("getPosition", (uint64_t (mrpt::io::CFileInputStream::*)() const) &mrpt::io::CFileInputStream::getPosition, "C++: mrpt::io::CFileInputStream::getPosition() const --> uint64_t");
		cl.def("readLine", (bool (mrpt::io::CFileInputStream::*)(std::string &)) &mrpt::io::CFileInputStream::readLine, "Reads one string line from the file (until a new-line character)\n \n\n true if a line has been read, false on EOF or error. \n\nC++: mrpt::io::CFileInputStream::readLine(std::string &) --> bool", pybind11::arg("str"));
		cl.def("Read", (size_t (mrpt::io::CFileInputStream::*)(void *, size_t)) &mrpt::io::CFileInputStream::Read, "C++: mrpt::io::CFileInputStream::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Write", (size_t (mrpt::io::CFileInputStream::*)(const void *, size_t)) &mrpt::io::CFileInputStream::Write, "C++: mrpt::io::CFileInputStream::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
	}
	{ // mrpt::io::CFileOutputStream file:mrpt/io/CFileOutputStream.h line:24
		pybind11::class_<mrpt::io::CFileOutputStream, std::shared_ptr<mrpt::io::CFileOutputStream>, PyCallBack_mrpt_io_CFileOutputStream, mrpt::io::CStream> cl(M("mrpt::io"), "CFileOutputStream", "This CStream derived class allow using a file as a write-only, binary\n stream.\n\n \n CStream, CFileStream, CFileGZOutputStream\n \n\n\n ");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::io::CFileOutputStream(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_io_CFileOutputStream(a0); } ), "doc");
		cl.def( pybind11::init<const std::string &, const enum mrpt::io::OpenMode>(), pybind11::arg("fileName"), pybind11::arg("mode") );

		cl.def( pybind11::init( [](){ return new mrpt::io::CFileOutputStream(); }, [](){ return new PyCallBack_mrpt_io_CFileOutputStream(); } ) );
		cl.def("open", [](mrpt::io::CFileOutputStream &o, const std::string & a0) -> bool { return o.open(a0); }, "", pybind11::arg("fileName"));
		cl.def("open", (bool (mrpt::io::CFileOutputStream::*)(const std::string &, const enum mrpt::io::OpenMode)) &mrpt::io::CFileOutputStream::open, "C++: mrpt::io::CFileOutputStream::open(const std::string &, const enum mrpt::io::OpenMode) --> bool", pybind11::arg("fileName"), pybind11::arg("mode"));
		cl.def("close", (void (mrpt::io::CFileOutputStream::*)()) &mrpt::io::CFileOutputStream::close, "Close the stream. \n\nC++: mrpt::io::CFileOutputStream::close() --> void");
		cl.def("getStreamDescription", (std::string (mrpt::io::CFileOutputStream::*)() const) &mrpt::io::CFileOutputStream::getStreamDescription, "C++: mrpt::io::CFileOutputStream::getStreamDescription() const --> std::string");
		cl.def("fileOpenCorrectly", (bool (mrpt::io::CFileOutputStream::*)() const) &mrpt::io::CFileOutputStream::fileOpenCorrectly, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileOutputStream::fileOpenCorrectly() const --> bool");
		cl.def("is_open", (bool (mrpt::io::CFileOutputStream::*)()) &mrpt::io::CFileOutputStream::is_open, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileOutputStream::is_open() --> bool");
		cl.def("Seek", [](mrpt::io::CFileOutputStream &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg("Offset"));
		cl.def("Seek", (uint64_t (mrpt::io::CFileOutputStream::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::io::CFileOutputStream::Seek, "C++: mrpt::io::CFileOutputStream::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg("Offset"), pybind11::arg("Origin"));
		cl.def("getTotalBytesCount", (uint64_t (mrpt::io::CFileOutputStream::*)() const) &mrpt::io::CFileOutputStream::getTotalBytesCount, "Method for getting the total number of bytes written to buffer \n\nC++: mrpt::io::CFileOutputStream::getTotalBytesCount() const --> uint64_t");
		cl.def("getPosition", (uint64_t (mrpt::io::CFileOutputStream::*)() const) &mrpt::io::CFileOutputStream::getPosition, "Method for getting the current cursor position, where 0 is the first\n byte and TotalBytesCount-1 the last one \n\nC++: mrpt::io::CFileOutputStream::getPosition() const --> uint64_t");
		cl.def("Read", (size_t (mrpt::io::CFileOutputStream::*)(void *, size_t)) &mrpt::io::CFileOutputStream::Read, "C++: mrpt::io::CFileOutputStream::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Write", (size_t (mrpt::io::CFileOutputStream::*)(const void *, size_t)) &mrpt::io::CFileOutputStream::Write, "C++: mrpt::io::CFileOutputStream::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
	}
}
