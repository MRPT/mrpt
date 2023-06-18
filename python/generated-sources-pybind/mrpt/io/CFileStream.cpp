#include <iterator>
#include <memory>
#include <mrpt/io/CFileStream.h>
#include <mrpt/io/CStream.h>
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

// mrpt::io::CFileStream file:mrpt/io/CFileStream.h line:38
struct PyCallBack_mrpt_io_CFileStream : public mrpt::io::CFileStream {
	using mrpt::io::CFileStream::CFileStream;

	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileStream *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileStream::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileStream *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CFileStream::Write(a0, a1);
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileStream *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileStream::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileStream *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileStream::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileStream *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CFileStream::getPosition();
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileStream *>(this), "ReadBufferImmediate");
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
	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CFileStream *>(this), "getStreamDescription");
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

void bind_mrpt_io_CFileStream(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::io::CFileStream file:mrpt/io/CFileStream.h line:38
		pybind11::class_<mrpt::io::CFileStream, std::shared_ptr<mrpt::io::CFileStream>, PyCallBack_mrpt_io_CFileStream, mrpt::io::CStream> cl(M("mrpt::io"), "CFileStream", "This CStream derived class allow using a file as a read/write binary stream,\n creating it if the file didn't exist.\n   The default behavior can be change to open as read, write, read and\n write,... in the constructor.\n \n\n CStream, CFileInputStream, CFileOutputStrea, CFileGZInputStream\n \n\n\n ");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::io::CFileStream(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_io_CFileStream(a0); } ), "doc");
		cl.def( pybind11::init<const std::string &, int>(), pybind11::arg("fileName"), pybind11::arg("mode") );

		cl.def( pybind11::init( [](){ return new mrpt::io::CFileStream(); }, [](){ return new PyCallBack_mrpt_io_CFileStream(); } ) );
		cl.def("Read", (size_t (mrpt::io::CFileStream::*)(void *, size_t)) &mrpt::io::CFileStream::Read, "C++: mrpt::io::CFileStream::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Write", (size_t (mrpt::io::CFileStream::*)(const void *, size_t)) &mrpt::io::CFileStream::Write, "C++: mrpt::io::CFileStream::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("open", [](mrpt::io::CFileStream &o, const std::string & a0) -> bool { return o.open(a0); }, "", pybind11::arg("fileName"));
		cl.def("open", (bool (mrpt::io::CFileStream::*)(const std::string &, int)) &mrpt::io::CFileStream::open, "Opens the file, returning true on success.\n \n\n The file to be open in this stream\n \n\n The open mode: can be an or'd conbination of different\n values.\n  By default the file is opened for open and write and created if not\n found.\n\nC++: mrpt::io::CFileStream::open(const std::string &, int) --> bool", pybind11::arg("fileName"), pybind11::arg("mode"));
		cl.def("close", (void (mrpt::io::CFileStream::*)()) &mrpt::io::CFileStream::close, "Closes the file \n\nC++: mrpt::io::CFileStream::close() --> void");
		cl.def("fileOpenCorrectly", (bool (mrpt::io::CFileStream::*)() const) &mrpt::io::CFileStream::fileOpenCorrectly, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileStream::fileOpenCorrectly() const --> bool");
		cl.def("is_open", (bool (mrpt::io::CFileStream::*)()) &mrpt::io::CFileStream::is_open, "Returns true if the file was open without errors. \n\nC++: mrpt::io::CFileStream::is_open() --> bool");
		cl.def("checkEOF", (bool (mrpt::io::CFileStream::*)()) &mrpt::io::CFileStream::checkEOF, "Will be true if EOF has been already reached. \n\nC++: mrpt::io::CFileStream::checkEOF() --> bool");
		cl.def("clearError", (void (mrpt::io::CFileStream::*)()) &mrpt::io::CFileStream::clearError, "Resets stream error status bits (e.g. after an EOF) \n\nC++: mrpt::io::CFileStream::clearError() --> void");
		cl.def("Seek", [](mrpt::io::CFileStream &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg("off"));
		cl.def("Seek", (uint64_t (mrpt::io::CFileStream::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::io::CFileStream::Seek, "C++: mrpt::io::CFileStream::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg("off"), pybind11::arg("org"));
		cl.def("getTotalBytesCount", (uint64_t (mrpt::io::CFileStream::*)() const) &mrpt::io::CFileStream::getTotalBytesCount, "C++: mrpt::io::CFileStream::getTotalBytesCount() const --> uint64_t");
		cl.def("getPosition", (uint64_t (mrpt::io::CFileStream::*)() const) &mrpt::io::CFileStream::getPosition, "C++: mrpt::io::CFileStream::getPosition() const --> uint64_t");
		cl.def("getPositionI", (uint64_t (mrpt::io::CFileStream::*)()) &mrpt::io::CFileStream::getPositionI, "The current Input cursor position, where 0 is the first byte \n\nC++: mrpt::io::CFileStream::getPositionI() --> uint64_t");
		cl.def("getPositionO", (uint64_t (mrpt::io::CFileStream::*)()) &mrpt::io::CFileStream::getPositionO, "The current Input cursor position, where 0 is the first byte \n\nC++: mrpt::io::CFileStream::getPositionO() --> uint64_t");
		cl.def("readLine", (bool (mrpt::io::CFileStream::*)(std::string &)) &mrpt::io::CFileStream::readLine, "Reads one string line from the file (until a new-line character)\n \n\n true if a line has been read, false on EOF or error \n\nC++: mrpt::io::CFileStream::readLine(std::string &) --> bool", pybind11::arg("str"));
	}
}
