#include <iterator>
#include <memory>
#include <mrpt/io/CPipe.h>
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

// mrpt::io::CPipeBaseEndPoint file:mrpt/io/CPipe.h line:62
struct PyCallBack_mrpt_io_CPipeBaseEndPoint : public mrpt::io::CPipeBaseEndPoint {
	using mrpt::io::CPipeBaseEndPoint::CPipeBaseEndPoint;

	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeBaseEndPoint *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPipeBaseEndPoint::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeBaseEndPoint *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPipeBaseEndPoint::Write(a0, a1);
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeBaseEndPoint *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeBaseEndPoint *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeBaseEndPoint *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::getPosition();
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeBaseEndPoint *>(this), "ReadBufferImmediate");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeBaseEndPoint *>(this), "getStreamDescription");
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

// mrpt::io::CPipeReadEndPoint file:mrpt/io/CPipe.h line:126
struct PyCallBack_mrpt_io_CPipeReadEndPoint : public mrpt::io::CPipeReadEndPoint {
	using mrpt::io::CPipeReadEndPoint::CPipeReadEndPoint;

	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeReadEndPoint *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPipeReadEndPoint::Write(a0, a1);
	}
	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeReadEndPoint *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPipeBaseEndPoint::Read(a0, a1);
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeReadEndPoint *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeReadEndPoint *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeReadEndPoint *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::getPosition();
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeReadEndPoint *>(this), "ReadBufferImmediate");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeReadEndPoint *>(this), "getStreamDescription");
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

// mrpt::io::CPipeWriteEndPoint file:mrpt/io/CPipe.h line:149
struct PyCallBack_mrpt_io_CPipeWriteEndPoint : public mrpt::io::CPipeWriteEndPoint {
	using mrpt::io::CPipeWriteEndPoint::CPipeWriteEndPoint;

	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeWriteEndPoint *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPipeWriteEndPoint::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeWriteEndPoint *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CPipeBaseEndPoint::Write(a0, a1);
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeWriteEndPoint *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeWriteEndPoint *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeWriteEndPoint *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CPipeBaseEndPoint::getPosition();
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeWriteEndPoint *>(this), "ReadBufferImmediate");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::io::CPipeWriteEndPoint *>(this), "getStreamDescription");
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

void bind_mrpt_io_CPipe(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::io::CPipeBaseEndPoint file:mrpt/io/CPipe.h line:62
		pybind11::class_<mrpt::io::CPipeBaseEndPoint, std::shared_ptr<mrpt::io::CPipeBaseEndPoint>, PyCallBack_mrpt_io_CPipeBaseEndPoint, mrpt::io::CStream> cl(M("mrpt::io"), "CPipeBaseEndPoint", "Common interface of read & write pipe end-points\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::io::CPipeBaseEndPoint(); }, [](){ return new PyCallBack_mrpt_io_CPipeBaseEndPoint(); } ) );
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("serialized") );

		cl.def_readwrite("timeout_read_start_us", &mrpt::io::CPipeBaseEndPoint::timeout_read_start_us);
		cl.def_readwrite("timeout_read_between_us", &mrpt::io::CPipeBaseEndPoint::timeout_read_between_us);
		cl.def("serialize", (std::string (mrpt::io::CPipeBaseEndPoint::*)()) &mrpt::io::CPipeBaseEndPoint::serialize, "Converts the end-point into a string suitable for reconstruction at a\n child process.\n This *invalidates* this object, since only one real end-point can exist\n at once.\n\nC++: mrpt::io::CPipeBaseEndPoint::serialize() --> std::string");
		cl.def("isOpen", (bool (mrpt::io::CPipeBaseEndPoint::*)() const) &mrpt::io::CPipeBaseEndPoint::isOpen, "Returns false if the pipe was closed due to some error. \n\nC++: mrpt::io::CPipeBaseEndPoint::isOpen() const --> bool");
		cl.def("close", (void (mrpt::io::CPipeBaseEndPoint::*)()) &mrpt::io::CPipeBaseEndPoint::close, "Closes the pipe (normally not needed to be called by users,\n automatically done at destructor) \n\nC++: mrpt::io::CPipeBaseEndPoint::close() --> void");
		cl.def("Read", (size_t (mrpt::io::CPipeBaseEndPoint::*)(void *, size_t)) &mrpt::io::CPipeBaseEndPoint::Read, "C++: mrpt::io::CPipeBaseEndPoint::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Write", (size_t (mrpt::io::CPipeBaseEndPoint::*)(const void *, size_t)) &mrpt::io::CPipeBaseEndPoint::Write, "C++: mrpt::io::CPipeBaseEndPoint::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Seek", [](mrpt::io::CPipeBaseEndPoint &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg("of"));
		cl.def("Seek", (uint64_t (mrpt::io::CPipeBaseEndPoint::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::io::CPipeBaseEndPoint::Seek, "Without effect in this class \n\nC++: mrpt::io::CPipeBaseEndPoint::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg("of"), pybind11::arg("o"));
		cl.def("getTotalBytesCount", (uint64_t (mrpt::io::CPipeBaseEndPoint::*)() const) &mrpt::io::CPipeBaseEndPoint::getTotalBytesCount, "Without effect in this class \n\nC++: mrpt::io::CPipeBaseEndPoint::getTotalBytesCount() const --> uint64_t");
		cl.def("getPosition", (uint64_t (mrpt::io::CPipeBaseEndPoint::*)() const) &mrpt::io::CPipeBaseEndPoint::getPosition, "Without effect in this class \n\nC++: mrpt::io::CPipeBaseEndPoint::getPosition() const --> uint64_t");
	}
	{ // mrpt::io::CPipeReadEndPoint file:mrpt/io/CPipe.h line:126
		pybind11::class_<mrpt::io::CPipeReadEndPoint, std::shared_ptr<mrpt::io::CPipeReadEndPoint>, PyCallBack_mrpt_io_CPipeReadEndPoint, mrpt::io::CPipeBaseEndPoint> cl(M("mrpt::io"), "CPipeReadEndPoint", "The read end-point in a pipe created with mrpt::synch::CPipe.\n Use the method CStream::Read() of the base class CStream\n for blocking reading.\n \n\n\n ");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("serialized") );

		cl.def("Write", (size_t (mrpt::io::CPipeReadEndPoint::*)(const void *, size_t)) &mrpt::io::CPipeReadEndPoint::Write, "Read-only pipe, don't call this method \n\nC++: mrpt::io::CPipeReadEndPoint::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
	}
	{ // mrpt::io::CPipeWriteEndPoint file:mrpt/io/CPipe.h line:149
		pybind11::class_<mrpt::io::CPipeWriteEndPoint, std::shared_ptr<mrpt::io::CPipeWriteEndPoint>, PyCallBack_mrpt_io_CPipeWriteEndPoint, mrpt::io::CPipeBaseEndPoint> cl(M("mrpt::io"), "CPipeWriteEndPoint", "The write end-point in a pipe created with mrpt::synch::CPipe.\n Use the method CStream::Write() of the base class CStream\n for blocking writing. ");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("serialized") );

		cl.def("Read", (size_t (mrpt::io::CPipeWriteEndPoint::*)(void *, size_t)) &mrpt::io::CPipeWriteEndPoint::Read, "Write-only pipe: read launches exception \n\nC++: mrpt::io::CPipeWriteEndPoint::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
	}
}
